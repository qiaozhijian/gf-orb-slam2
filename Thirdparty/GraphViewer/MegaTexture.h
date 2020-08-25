/*
								+--------------------------------+
								|                                |
								|     ***  Mega-texture  ***     |
								|                                |
								|  Copyright © -tHE SWINe- 2016  |
								|                                |
								|         MegaTexture.h          |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __MEGATEXTURE_V0_INCLUDED
#define __MEGATEXTURE_V0_INCLUDED

/**
 *	@file MegaTexture.h
 *	@brief simple virtual texturing and huge texture atlases
 *	@author -tHE SWINe-
 *	@date 2016
 */

#include "../UberLame_src/NewFix.h"
#include "../UberLame_src/CallStack.h"
#include "../UberLame_src/gles2/GLES20Emu.h"
typedef double GLdouble; // needed in glu.h, included through glut.h
#ifdef __APPLE__
#include <GLUT/glut.h>
#else // __APPLE__
#include <GL/glut.h>
#endif // __APPLE__
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <string>
#include <algorithm>
#include <set>
#include <map>
#include "../UberLame_src/BinPacking.h"
#include "../UberLame_src/Vector.h"
#include "../UberLame_src/StlUtils.h"
#include "../UberLame_src/Segregated.h"
#include "../UberLame_src/gles2/Texture.h"
#include "../UberLame_src/gles2/BufferObjects.h"
#include "../UberLame_src/gles2/Shader.h"
#include "../UberLame_src/gles2/TextureUtil.h"
#include "../UberLame_src/gles2/FrameBuffer.h"
#include "../UberLame_src/gles2/Transform.h"
#include "../UberLame_src/Integer.h"
#include "../UberLame_src/Dir.h"
#include "../UberLame_src/Hilbert.h"

#if !defined(_WIN32) && !defined(_WIN64)
#include <time.h>
inline void Sleep(int n_milliseconds)
{
	struct timespec tim;
	tim.tv_sec = n_milliseconds / 1000;
	tim.tv_nsec = (n_milliseconds % 1000) * 1000000;
	nanosleep(&tim, 0);
}
#endif // !_WIN32 && !_WIN64

class CImageInfo {
protected:
	std::vector<std::string> m_filename_list;
	std::vector<TBmp> m_info_list;
	size_t m_n_total_texel_num;
	size_t m_n_failed_image_num;

public:
	CImageInfo(const std::vector<std::string> &r_filename_list) // throw(std::bad_alloc)
		:m_filename_list(r_filename_list), m_info_list(r_filename_list.size()), // copy
		m_n_total_texel_num(0), m_n_failed_image_num(0)
	{
		size_t n = m_filename_list.size();
		_ASSERTE(n <= INT_MAX);
		int _n = (int)n;
		#pragma omp parallel for schedule(dynamic, 1)
		for(int i = 0; i < _n; ++ i) {
			TBmp t_bmp = CGLESTextureLoader::t_GetImageInfo(m_filename_list[i].c_str());
			m_info_list[i] = t_bmp;
			size_t n_image_size = t_bmp.n_width * t_bmp.n_height;
			#pragma omp atomic
			m_n_total_texel_num += n_image_size;
			if(!n_image_size) {
				#pragma omp atomic
				++ m_n_failed_image_num;
			}
		}
	}

	size_t n_Pixel_Num() const
	{
		return m_n_total_texel_num;
	}

	size_t n_Image_Num() const
	{
		return m_info_list.size();
	}

	size_t n_FailedImage_Num() const
	{
		return m_n_failed_image_num;
	}

	const TBmp &t_ImageInfo(size_t n_image) const
	{
		return m_info_list[n_image];
	}

	const char *p_s_ImageFilename(size_t n_image) const
	{
		return m_filename_list[n_image].c_str();
	}

	const std::string &s_ImageFilename(size_t n_image) const
	{
		return m_filename_list[n_image];
	}
};

#ifdef _MSC_VER
#pragma warning(disable: 4503)
#endif

#include <queue>
#include "LRU.h"
#include "../UberLame_src/Thread.h"

class CAsyncImageLoader {
protected:
	struct TImageInfo {
		std::string s_filename;
		bool *p_ready_flag; // pointer to the ready-flag
		TBmp *p_image_data;
		bool b_corrected_size_discrepancy;
		size_t n_target_width, n_target_height; // will support minification in the future // todo
		bool b_loading_finished; // set once the loading finished; if p_image_data is nonzero then it alzo succeeded
		bool b_locked; // if the caller is looking at the image, it is locked and cannot be deleted
	};

	template <class TItem = TImageInfo*>
	class CImageQueue { // a queue with variable size where the producer does not wait but the consumer waits
	protected:
		CMutex m_sync;
		CSemaphore m_fill_sema;
		std::queue<TItem> m_queue;
		bool m_b_finished;

	public:
		CImageQueue()
			:m_fill_sema(0), m_b_finished(false)
		{}

		~CImageQueue()
		{
			Signal_Finished();
		}

		bool b_Status() const
		{
			return m_sync.b_Status() && m_fill_sema.b_Status();
		}

		bool b_Empty() const // async, cannot be relied upon (only informative)
		{
			return m_queue.empty();
		}

		size_t n_Size() const // async, cannot be relied upon (only informative)
		{
			return m_queue.size();
		}

		bool b_Finished() const
		{
			return b_Empty() && m_b_finished;
		}

		bool b_Finish_Signaled() const
		{
			return m_b_finished;
		}

		bool Signal_Finished()
		{
			if(m_b_finished)
				return true;
			m_b_finished = true;
			return m_fill_sema.Signal();
		}

		bool Put(TItem t_item) // puts an item, returns immediately
		{
			if(m_b_finished)
				return false; // not anymore
			if(!m_sync.Lock())
				return false;

			try {
				m_queue.push(t_item);
			} catch(std::bad_alloc&) {
				m_sync.Unlock(); // make sure to unlock
				return false;
			}
			// push the item into the queue

			bool b_signal_success = m_fill_sema.Signal();
			return m_sync.Unlock() && b_signal_success;
			// make sure to unlock, even on failure
		}

		bool Get(TItem &r_t_item, bool &r_b_got_item) // gets an item, waits indefinitely
		{
			if(!m_fill_sema.Wait())
				return false;
			// wait for a single item to be produced

			if(!m_sync.Lock())
				return false;

			if(m_b_finished && m_queue.empty()) {
				r_b_got_item = false;
				bool b_fill_signalled = m_fill_sema.Signal(); // signal the semaphore in case there are more threads waiting
				return m_sync.Unlock() && b_fill_signalled;
			} else
				r_b_got_item = true;

			_ASSERTE(!m_queue.empty());
			r_t_item = m_queue.front(); // if this throws, we lose the mutex, this takes assumptions about the user types
			m_queue.pop();
			// get an item

			return m_sync.Unlock();
		}

	protected:
		CImageQueue(const CImageQueue &r_other); // no-copy
		CImageQueue &operator =(const CImageQueue &r_other); // no-copy
	};

	class CImageLoader : public CRunable_Thread_ShallowCopy {
	protected:
		CImageQueue<> *m_p_queue;

	public:
		CImageLoader(CImageQueue<> &r_queue)
			:m_p_queue(&r_queue)
		{}

		CImageLoader(const CImageLoader &r_other)
			:m_p_queue(r_other.m_p_queue)
		{}

		/*CImageLoader & operator =(const CImageLoader &r_other)
		{
			m_r_queue(r_other.m_r_queue)
			return *this;
		}*/

	protected:
		virtual void Run()
		{
			CImageQueue<> &m_r_queue = *m_p_queue;
			TImageInfo *p_image_to_load;
			bool b_got_work;
			while(m_r_queue.Get(p_image_to_load, b_got_work)) {
				if(!b_got_work) {
					_ASSERTE(m_r_queue.b_Finished()); // should be the first thread out of work
					return;
				}

				_ASSERTE(!p_image_to_load->p_image_data);

				p_image_to_load->p_image_data = CGLESTextureLoader::p_LoadImage(p_image_to_load->s_filename.c_str());
				// load the image

				p_image_to_load->b_loading_finished = true; // finished, may have failed
				// initialize the structure

				if(p_image_to_load->p_ready_flag)
					*(p_image_to_load->p_ready_flag) = true;
				// signal loaded to the originator of the request
			}
		}
	};

	class CIterHashAdapter {
	public:
		template <class _Kty>
		size_t operator()(const _Kty &_Keyval) const
		{
			return std::tr1::hash<ptrdiff_t>()((ptrdiff_t)&(*_Keyval));
			// todo - linux / mac version
		}
	};

	typedef std::multimap<std::string, TImageInfo> CImageMap;
	typedef CImageMap::iterator CImageIt;
	typedef CLRU_Set<CImageIt, CIterHashAdapter> CCachePolicy;

	class CCompareBitmapPtr {
	protected:
		const TBmp *m_p_image;

	public:
		CCompareBitmapPtr(const TBmp *p_image)
			:m_p_image(p_image)
		{}

		bool operator ()(const CImageMap::value_type &r_t_image) const
		{
			return r_t_image.second.b_loading_finished &&
				r_t_image.second.p_image_data == m_p_image;
		}
	};

protected:
	CCachePolicy m_image_priority;
	CImageMap m_image_list;
	const size_t m_n_max_resident_memory;
	size_t m_n_resident_memory;
	CImageQueue<> m_request_queue;
	std::vector<CImageLoader> m_worker_list;
	bool m_b_workers_running;

	size_t m_n_loaded_image_num, m_n_cache_hits;
	uint64_t m_n_cumulative_loaded_memory;
	std::set<std::string> m_unique_image_set;
	// stats

public:
	CAsyncImageLoader(size_t n_max_resident_memory = 1024 * 1048576, size_t n_max_loader_threads = 1)
		:m_n_max_resident_memory(n_max_resident_memory), m_n_resident_memory(0), m_b_workers_running(false),
		m_n_loaded_image_num(0), m_n_cache_hits(0), m_n_cumulative_loaded_memory(0)
	{
		stl_ut::Resize_To_N(m_worker_list, n_max_loader_threads, CImageLoader(m_request_queue));
	}

	~CAsyncImageLoader()
	{
		Show_Stats();
		// maybe a good idea for debugging

		m_request_queue.Signal_Finished();
		// stop the queue, free the workers to finish

		for(size_t i = 0, n = m_worker_list.size(); i < n; ++ i)
			m_worker_list[i].WaitForFinish();
		// join the workers

		m_image_priority.Erase_Back(m_image_priority.n_Size());
		// erase the queue

		std::for_each(m_image_list.begin(), m_image_list.end(), DeleteImage);
		// delete all the images
	}

	bool Start_Workers()
	{
		if(m_b_workers_running)		
			return true;
		for(size_t i = 0, n = m_worker_list.size(); i < n; ++ i) {
			if(m_worker_list[i].Start())
				m_b_workers_running = true;
		}
		return m_b_workers_running;
	}

	size_t n_ResidentImage_Num() const
	{
		_ASSERTE(m_image_priority.n_Size() == m_image_list.size());
		return m_image_list.size();
	}

	size_t n_ResidentImage_Bytes() const
	{
		return m_n_resident_memory;
	}

	bool Enqueue_FetchImage(const std::string &r_s_filename,
		size_t n_target_width, size_t n_target_height, bool *p_ready_flag)
	{
		if(p_ready_flag)
			*p_ready_flag = false;

		if(!Start_Workers())
			return false;
		// make sure there is someone to handle the requests

		std::pair<CImageIt, CImageIt> image_range = m_image_list.equal_range(r_s_filename);
		CImageIt p_next_bigger_res_it = image_range.second;
		size_t n_next_bigger_image_size = size_t(-1);
		for(; image_range.first != image_range.second; ++ image_range.first) {
			TImageInfo &r_t_image = (*image_range.first).second;
			if(r_t_image.n_target_width == n_target_width &&
			   r_t_image.n_target_height == n_target_height) {
				if(r_t_image.b_loading_finished) {
					m_image_priority.Insert(image_range.first);
					// increase cache position

					++ m_n_cache_hits;

					if(p_ready_flag)
						*p_ready_flag = true;
					return true;
				} else {
					/*if(!p_ready_flag)
						return true;*/ // not ok, will load but when the first caller returns it, it could be evicted before the second caller can get it
					fprintf(stderr, "error: image \'%s\' requested twice before"
						" it could be loaded\n", r_s_filename.c_str());
					return false;
					// well, we're screwed
				}
			} else if(r_t_image.n_target_width >= n_target_width &&
			   r_t_image.n_target_height >= n_target_height) {
				size_t n_image_size = r_t_image.n_target_width * r_t_image.n_target_height;
				if(n_next_bigger_image_size > n_image_size) {
					p_next_bigger_res_it = image_range.first;
					n_next_bigger_image_size = n_image_size;
				}
				// see if there is a bigger one, it should still be faster than going to the disk
			}
		}
		// see if there is an exact match of the image

		if(p_next_bigger_res_it != image_range.second) {
			TImageInfo &r_t_image = (*p_next_bigger_res_it).second;
			if(r_t_image.b_loading_finished) {
				m_image_priority.Insert(p_next_bigger_res_it);
				// increase cache position

				++ m_n_cache_hits;

				if(p_ready_flag)
					*p_ready_flag = true;
				return true;
			} else {
				fprintf(stderr, "error: image \'%s\' requested twice before"
					" it could be loaded\n", r_s_filename.c_str());
				return false;
				// well, we're screwed
			}
		}
		// see if we found a bigger one (maybe schedule rescaling jobs as well?)

		bool b_had_to_delete = false;
		const size_t n_image_size = n_target_width * n_target_height * sizeof(uint32_t);
		if(m_n_resident_memory + n_image_size - min(m_n_max_resident_memory, n_image_size) >
		   m_n_max_resident_memory - min(m_n_max_resident_memory, n_image_size)) { // watch out for overflows
			const size_t n_size_to_delete = m_n_resident_memory + n_image_size - m_n_max_resident_memory;
			// should have no overflows here (can have one overflow and one underflow
			// that fix each other since this is unsigned)

			size_t n_deleted_size = n_TryDelete(n_size_to_delete);
			b_had_to_delete = n_deleted_size > 0;
			if(n_deleted_size < n_size_to_delete) {
				fprintf(stderr, "error: image \'%s\' failed to load, could't free up "
					PRIsizeB "B\n", r_s_filename.c_str(), PRIsizeBparams(n_size_to_delete));
				return false;
			}
		}
		// first need to free up some images

		try {
			TImageInfo t_image;
			t_image.b_loading_finished = false;
			t_image.b_locked = true; // !! the images must be initially locked otherwise they could be purged before actually being used
			t_image.n_target_width = n_target_width;
			t_image.n_target_height = n_target_height;
			t_image.b_corrected_size_discrepancy = false;
			t_image.p_image_data = 0;
			t_image.p_ready_flag = p_ready_flag;
			t_image.s_filename = r_s_filename;
			CImageIt p_new_it = (b_had_to_delete)? m_image_list.insert(std::make_pair(r_s_filename, t_image)) : // image range may have been invalidated
				m_image_list.insert(image_range.second, std::make_pair(r_s_filename, t_image));
			if(!m_image_priority.Insert(p_new_it)) {
				m_image_list.erase(p_new_it); // keep it consistent
				return false;
			}

			bool b_result = m_request_queue.Put(&(*p_new_it).second);
			if(b_result) {
				m_n_resident_memory += n_image_size;

				m_unique_image_set.insert(r_s_filename); // see if this is prohibitively costly
				++ m_n_loaded_image_num;
				m_n_cumulative_loaded_memory += n_image_size;
				// all-time stats
			}
			// this might add size too soon; could have another return queue and count the size
			// at the point when the image is returning; this will exhaust the memory in the long run
			// if the images are resized inexactly

			/*std::pair<const char*, const char*> t_fn = CPath::t_ShortFileName(r_s_filename.c_str());
			printf("debug: queued \'%s%s\'\n", t_fn.first, t_fn.second);
			// verbose*/

			return b_result;
		} catch(std::bad_alloc&) {
			return false;
		}
		// add the image into the list
	}

	const TBmp *p_LookupImage(const std::string &r_s_filename,
		size_t n_target_width, size_t n_target_height,
		bool b_increase_MRU = true, bool b_asynchronous = true) // clear b_asynchronous to get the image immediately (e.g. when doing offscreen rendering)
	{
		std::pair<CImageIt, CImageIt> image_range = m_image_list.equal_range(r_s_filename);
		CImageIt p_next_bigger_res_it = image_range.second;
		size_t n_next_bigger_image_size = size_t(-1);
		for(; image_range.first != image_range.second; ++ image_range.first) {
			TImageInfo &r_t_image = (*image_range.first).second;
			if(r_t_image.n_target_width == n_target_width &&
			   r_t_image.n_target_height == n_target_height) {
				if(b_asynchronous && !r_t_image.b_loading_finished)
					return 0; // not loaded

				while(!r_t_image.b_loading_finished)
					Sleep(100); // todo - handle this in a better way (for now it will do) // t_odo - provide linux counterpart
				// otherwise wait for it to load

				_ASSERTE(!r_t_image.b_locked || !r_t_image.b_corrected_size_discrepancy); // only one lock; the first lock is by the loading thread and that one is smybolically released here
				r_t_image.b_locked = true;

				if(b_increase_MRU)
					m_image_priority.Insert(image_range.first);
				// increase cache position

				if(!r_t_image.b_corrected_size_discrepancy && r_t_image.p_image_data) {
					r_t_image.b_corrected_size_discrepancy = true;
					size_t n_actual_size = r_t_image.p_image_data->n_width *
						r_t_image.p_image_data->n_height * sizeof(uint32_t);
					size_t n_assumed_size = r_t_image.n_target_width *
						r_t_image.n_target_height * sizeof(uint32_t);
					_ASSERTE(m_n_resident_memory >= n_assumed_size);
					m_n_resident_memory -= n_assumed_size;
					m_n_resident_memory += n_actual_size;
					m_n_cumulative_loaded_memory -= n_assumed_size;
					m_n_cumulative_loaded_memory += n_actual_size;
					if(m_n_resident_memory > m_n_max_resident_memory)
						n_TryDelete(m_n_resident_memory - m_n_max_resident_memory); // ignore results though, now everything is loaded so it is too late now
				}
				// correct for discrepancies in image payload size

				return r_t_image.p_image_data;
			}
		}
		// see if there is an exact match of the image

		fprintf(stderr, "error: failed to look up an image\n");

		return 0;
	}
	// if b_asynchronous is set, then this only returns the image if r_b_ready_flag was already set, otherwise it returns 0
	// the image then remains allocated until Return_Image() is called
	// this might still fail if the b_asynchronous is cleared in case the maximum resident memory was exceeded and not enough images were returned

	void Return_Image(const TBmp *p_image, bool b_remain_resident = true)
	{
		CImageIt p_im = std::find_if(m_image_list.begin(), m_image_list.end(), CCompareBitmapPtr(p_image));
		_ASSERTE(p_im != m_image_list.end()); // this was returned before so it still better be there
		_ASSERTE((*p_im).second.b_locked); // this was locked before

		(*p_im).second.b_locked = false;
		// not locked, can be deleted ... in time

		if(!b_remain_resident) {
			// todo - implement this
		}
	}

	void Show_Stats() const
	{
		printf("debug: image loader loaded " PRIsize " images from the disk (" PRIsizeB "B in total)\n",
			m_n_loaded_image_num, PRIsizeBparams(m_n_cumulative_loaded_memory));
		printf("debug: image loader cache had " PRIsize " hits, saw " PRIsize
			" unique filenames (" PRIsize " were reloaded)\n", m_n_cache_hits,
			m_unique_image_set.size(), m_n_loaded_image_num - m_unique_image_set.size());
		printf("debug: image loader has " PRIsize " images cached or loading (" PRIsizeB "B)\n",
			m_image_list.size(), PRIsizeBparams(m_n_resident_memory));
#ifdef _DEBUG
		if(m_image_priority.n_Size() > 30) {
			CCachePolicy::CConstIterator p_end_it = m_image_priority.p_Begin_It();
			std::advance(p_end_it, 30);
			std::for_each(m_image_priority.p_Begin_It(), p_end_it, PrintImageInfo);
			printf("\t[and " PRIsize " more]\n", std::distance(p_end_it, m_image_priority.p_End_It()));
		} else
			std::for_each(m_image_priority.p_Begin_It(), m_image_priority.p_End_It(), PrintImageInfo);
#endif // _DEBUG
	}

protected:
	CAsyncImageLoader(const CAsyncImageLoader &r_other); // no-copy
	CAsyncImageLoader &operator =(const CAsyncImageLoader &r_other); // no-copy

	size_t n_TryDelete(size_t n_size_to_delete)
	{
		size_t n_deleted_size = 0;

		size_t n_delete_at_end = 0;
		CCachePolicy::CConstIterator p_last_entry = m_image_priority.p_End_It(),
			p_first_entry = m_image_priority.p_Begin_It();
		for(; p_last_entry != p_first_entry;) {
			-- p_last_entry; // here

			CImageIt p_image_it = *p_last_entry;
			TImageInfo &r_t_image = (*p_image_it).second;
			if(r_t_image.b_loading_finished && !r_t_image.b_locked) {
				++ n_delete_at_end;
				size_t n_delete_size = (r_t_image.p_image_data)? ((r_t_image.b_corrected_size_discrepancy)?
					r_t_image.p_image_data->n_width * r_t_image.p_image_data->n_height : r_t_image.n_target_width *
					r_t_image.n_target_height) * sizeof(uint32_t) : 0;

				r_t_image.p_image_data->Delete();
				m_image_priority.Erase_Back(1); //m_image_priority.Erase(p_image_it); // the same but requires no lookup this way
				m_image_list.erase(p_image_it);
				p_last_entry = m_image_priority.p_End_It(); // got invalidated, no guarantee that there are other elems
				p_first_entry = m_image_priority.p_Begin_It(); // may have gotten invalidated too
				n_deleted_size += n_delete_size;
				if(n_deleted_size >= n_size_to_delete)
					break;
			} else
				break;
		}
		//m_image_priority.Erase_Back(n_delete_at_end); // can't use, the hash entries (m_image_list iterators) must still be valid for the hash to work
		// delete consecutive run of images

		if(n_deleted_size < n_size_to_delete) {
			//p_first_entry = m_image_priority.p_Begin_It(); // got invalidated
			// note that this skips the last item, which is correct, also it was not deleted, so its iterator is still valid
			for(; p_last_entry != p_first_entry;) {
				-- p_last_entry; // here

				CImageIt p_image_it = *p_last_entry;
				TImageInfo &r_t_image = (*p_image_it).second;
				if(r_t_image.b_loading_finished && !r_t_image.b_locked) {
					size_t n_delete_size = (r_t_image.p_image_data)? ((r_t_image.b_corrected_size_discrepancy)?
						r_t_image.p_image_data->n_width * r_t_image.p_image_data->n_height : r_t_image.n_target_width *
						r_t_image.n_target_height) * sizeof(uint32_t) : 0;

					r_t_image.p_image_data->Delete();
					++ p_last_entry; // move away so that it does not get invalidated (there is at least one next item)
					m_image_priority.Erase(p_image_it);
					m_image_list.erase(p_image_it);
					n_deleted_size += n_delete_size;
					if(n_deleted_size >= n_size_to_delete)
						break;
				}
			}
		}
		// delete non-consecutive runs of images

#ifdef _DEBUG
		printf("debug: deleted " PRIsizeB "B worth of images from CPU cache   \n",
			PRIsizeBparams(n_deleted_size));
#endif // _DEBUG

		_ASSERTE(m_n_resident_memory >= n_deleted_size);
		m_n_resident_memory -= n_deleted_size;

		return n_deleted_size;
	}

	static void PrintImageInfo(CImageMap::iterator p_img_it)
	{
		size_t n_size = 0, n_width = (*p_img_it).second.n_target_width,
			n_height = (*p_img_it).second.n_target_height;
		if((*p_img_it).second.b_loading_finished && (*p_img_it).second.p_image_data) {
			n_size = (n_width = (*p_img_it).second.p_image_data->n_width) *
				(n_height = (*p_img_it).second.p_image_data->n_height) * sizeof(uint32_t);
		}
		std::pair<const char*, const char*> t_fn = CPath::t_ShortFileName((*p_img_it).first.c_str());
		printf("debug: \'%s%s\': %s (%d x %d, " PRIsizeB "B)\n",
			t_fn.first, t_fn.second, ((*p_img_it).second.b_loading_finished)?
			(((*p_img_it).second.b_locked)? "locked" : "cached") : "loading",
			(int)n_width, (int)n_height, PRIsizeBparams(n_size));
	}

	static void DeleteImage(CImageMap::value_type &r_t_img)
	{
		if(r_t_img.second.b_locked)
			fprintf(stderr, "warning: deleting images that are still locked\n");
		if(r_t_img.second.p_image_data) {
			r_t_img.second.p_image_data->Delete();
			r_t_img.second.p_image_data = 0;
		}
	}
};
// t_odo - implement this to load images in a fancy way while rendering

#ifndef GL_TEXTURE_MAX_LEVEL
#define GL_TEXTURE_MAX_LEVEL										0x813D /**< @brief OpenGL 1.2 enum */
#endif // !GL_TEXTURE_MAX_LEVEL

#include "../UberLame_src/BitmapFont2.h"

class CGLTextureScalingUploader {
protected:
	struct TRescaleShader : public CGLESShader {
		GLint n_texture_unit_uniform, n_pos_scale_uniform, n_tex_scale_uniform;

		bool Compile()
		{
			const char *p_s_vertex_shader =
				"precision highp float;\n"
				"\n"
				"attribute vec2 v_pos;\n"
				"\n"
				"uniform vec4 t_pos_scale;\n"
				"uniform vec4 t_tex_scale;\n"
				"\n"
				"varying vec2 v_texcoord;\n"
				"\n"
				"void main()\n"
				"{\n"
				"    gl_Position = vec4(t_pos_scale.xy + v_pos * t_pos_scale.zw, 0.5, 1.0);\n"
				"    v_texcoord = t_tex_scale.xy + v_pos * t_tex_scale.zw;\n"
				"}\n";
			const char *p_s_fragment_shader =
				"precision highp float;\n"
				"\n"
				"uniform sampler2D n_texture_unit;\n"
				"\n"
				"varying vec2 v_texcoord;\n"
				"\n"
				"void main()\n"
				"{\n"
				"    gl_FragColor = texture2D(n_texture_unit, v_texcoord);\n"
				"}\n";
			// vertex / fragment shader source code

			const char *p_s_config =
				"vertex {\n"
				"	v_pos: 0;\n" // pos always 0
				"}\n";
			// shader configuration (doesn't actually need the newlines)
			// note this can't override layout qualifiers in the shader code

			std::string compile_log, config_log, link_log;
			if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
			   p_s_config, compile_log, config_log, link_log, true))
				return false;
			// use the comfy function

			n_pos_scale_uniform = n_Get_Uniform_Location("t_pos_scale");
			n_tex_scale_uniform = n_Get_Uniform_Location("t_tex_scale");
			n_texture_unit_uniform = n_Get_Uniform_Location("n_texture_unit");
			// get addresses of the uniforms

			CGLESShader::Bind();
			// bind the shader

			Uniform1i(n_texture_unit_uniform, 0);
			// always assume the texture in unit 0

			return true;
		}

		void Bind(float f_x0, float f_y0, float f_w, float f_h,
			float f_s0, float f_t0, float f_s, float f_t) const
		{
			CGLESShader::Bind();
			// bind the shader

			Uniform4f(n_pos_scale_uniform, f_x0, f_y0, f_w, f_h);
			Uniform4f(n_tex_scale_uniform, f_s0, f_t0, f_s, f_t);
			// set the uniforms
		}
	};

	struct TTextShader : public CGLESShader {
		GLint n_modelview_matrix_uniform;
		GLint n_texture_unit_uniform;

		bool Compile()
		{
			const char *p_s_vertex_shader =
				"precision highp float;\n"
				"\n"
				"attribute vec3 v_tex;\n"
				"attribute vec2 v_pos;\n"
				"\n"
				"uniform mat4 t_modelview_projection_matrix;\n"
				"\n"
				"varying vec3 v_texcoord;\n"
				"\n"
				"void main()\n"
				"{\n"
				"    gl_Position = t_modelview_projection_matrix * vec4(v_pos, 0.0, 1.0);\n"
				"    v_texcoord = v_tex;\n"
				"}\n";
			const char *p_s_fragment_shader =
				"precision highp float;\n"
				"\n"
				//"#extension GL_EXT_texture_array : enable\n"
				"\n"
				"varying vec3 v_texcoord;\n"
				"\n"
				//"uniform sampler2DArray n_texture_unit;\n"
				"uniform sampler2D n_texture_unit;\n"
				"\n"
				"void main()\n"
				"{\n"
				//"    gl_FragColor = vec4(vec3(.0), texture2DArray(n_texture_unit, v_texcoord).x);\n"
				"    gl_FragColor = vec4(vec3(.0), texture2D(n_texture_unit, v_texcoord.xy).x);\n"
				"}\n";
			// vertex / fragment shader source code

			const char *p_s_config =
				"vertex {\n"
				"	v_tex: 0;\n"
				"	v_pos: 1;\n"
				"}\n";
			// shader configuration (doesn't actually need the newlines)
			// note this can't override layout qualifiers in the shader code

			std::string compile_log, config_log, link_log;
			if(!CompileConfigureLink(p_s_vertex_shader, p_s_fragment_shader,
			   p_s_config, compile_log, config_log, link_log, true))
				return false;
			// use the comfy function

			n_texture_unit_uniform = n_Get_Uniform_Location("n_texture_unit");
			n_modelview_matrix_uniform = n_Get_Uniform_Location("t_modelview_projection_matrix");
			// get addresses of the uniforms

			CGLESShader::Bind();
			// bind the shader

			Uniform1i(n_texture_unit_uniform, 0);
			// always assume the texture in unit 0

			return true;
		}

		void Bind(const Matrix4f &r_t_mvp)
		{
			CGLESShader::Bind();
			// bind the shader

			UniformMatrix4fv(n_modelview_matrix_uniform, 1, false, &r_t_mvp[0][0]);
			// set the projection matrix
		}
	};

protected:
	const GLenum m_n_internal_format;

	CGLFrameBufferObject m_fbo;
	CGLTexture_2D m_texture;
	CGLArraySetup m_quad;
	TRescaleShader m_shader;

	int m_n_vw, m_n_vh;
	GLenum m_n_last_min_filter;
	GLenum m_n_last_wrap_mode;
	int m_p_prev_wp[4];

	static const float p_quad[8];

	TTextShader m_font_shader;
	CGLBitmapFont2 m_font; // for rendering tile ids
	bool m_b_font_loaded;

public:
	CGLTextureScalingUploader(GLenum n_internal_format = GL_RGB)
		:m_n_internal_format(n_internal_format), m_fbo(8, 8, 1, true, 0, 0, false, false, 0, 0),
		m_texture(8, 8, n_internal_format, true), m_quad(p_quad, sizeof(p_quad), 0, GL_FLOAT, 2,
		0, GL_FLOAT, 0, 0, GL_TRIANGLE_STRIP), m_n_last_min_filter(GL_NEAREST),
		m_n_last_wrap_mode(GL_REPEAT), m_b_font_loaded(false)
	{
		m_shader.Compile();
		m_font_shader.Compile();
	}

	bool b_Status() const
	{
		return m_shader.b_Compiled() && m_shader.b_Linked() &&
			m_font_shader.b_Compiled() && m_font_shader.b_Linked() &&
			m_fbo.b_Status() && m_texture.b_Status() && m_quad.b_Status();
	}

	bool Set_Target(CGLTexture_2D &r_dest)
	{
		glGetIntegerv(GL_VIEWPORT, m_p_prev_wp);
		bool b_result = m_fbo.Bind();
		glViewport(0, 0, m_n_vw = r_dest.n_Width(), m_n_vh = r_dest.n_Height());
		m_fbo.Bind_ColorTexture(0, r_dest, 0);
		return b_result;
	}

	bool Set_Target(CGLTexture_2D_Array &r_dest, int n_layer)
	{
		glGetIntegerv(GL_VIEWPORT, m_p_prev_wp);
		bool b_result = m_fbo.Bind();
		glViewport(0, 0, m_n_vw = r_dest.n_Width(), m_n_vh = r_dest.n_Height());
		m_fbo.Bind_ColorTextureLayer(0, r_dest, 0, n_layer);
		return b_result;
	}

	/*static void RenderString(Vector3f v_pos, const char *p_s_string,
					  void *p_font = GLUT_BITMAP_8_BY_13)
	{
		glRasterPos2f(v_pos.x, v_pos.y); // deprecated
		for(; *p_s_string; ++ p_s_string)
    		glutBitmapCharacter(p_font, *p_s_string);
	}*/

	void Debug_FillTileImages(int n_tile_size, size_t n_first_tile_id)
	{
		bool b_loaded = false;
		bool b_have_font = m_b_font_loaded || (b_loaded = m_b_font_loaded =
			m_font.Load_2D("Noto Sans CJK kr Light_48_page-00.tga"/*"tahoma_48_page-00.tga"*/, true));
		if(b_loaded)
			printf("debug: loaded a font\n");
		// lazily load a font

		TBmp *p_bmp = TBmp::p_Alloc(n_tile_size, n_tile_size);
		p_bmp->Clear(0xffcccccc); // clear with light grey
		p_bmp->DrawLine_AA2(.0f, .0f, n_tile_size, n_tile_size, 0xff888888, 1, 0); // diag line
		p_bmp->DrawLine_AA2(.0f, n_tile_size, n_tile_size, .0f, 0xff888888, 1, 0); // diag line
		p_bmp->DrawLine_AA2(.0f, n_tile_size * .5f, n_tile_size, n_tile_size * .5f, 0xff888888, 1, 0); // horizontal line
		p_bmp->DrawLine_AA2(n_tile_size * .5f, .0f, n_tile_size * .5f, n_tile_size, 0xff888888, 1, 0); // vertical line
		p_bmp->DrawRect(0, 0, n_tile_size - 1, n_tile_size - 1, 0xff444444); // dark gray frame

		GLenum n_repeat_mode = GL_CLAMP_TO_EDGE;
		int n_max_levels = 0;

		glActiveTexture(GL_TEXTURE0); // ...
		m_texture.Bind();
		GLenum n_min_filter = (n_max_levels >= 1)? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR;
		if(m_n_last_min_filter != n_min_filter) {
			m_texture.Set_Min_Filter(n_min_filter);
			m_n_last_min_filter = n_min_filter;
		}
		glTexImage2D(m_texture.n_Target(), 0, m_n_internal_format, p_bmp->n_width,
			p_bmp->n_height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, p_bmp->p_buffer);
		glTexParameteri(m_texture.n_Target(), GL_TEXTURE_MAX_LEVEL, n_max_levels); // todo - time if this makes any difference
		if(n_max_levels >= 1)
			glGenerateMipmap(m_texture.n_Target());
		if(m_n_last_wrap_mode != n_repeat_mode) {
			m_texture.Set_Wrap_S(n_repeat_mode);
			m_texture.Set_Wrap_T(n_repeat_mode);
			m_n_last_wrap_mode = n_repeat_mode;
		}
		// upload the texture

		p_bmp->Delete();

		std::vector<float> font_vertex_list; // t3f v2f
		std::vector<uint32_t> font_index_list;

		float f_page_aspect = float(m_n_vh) / m_n_vw;
		// fix the atlas page aspect ratio

		float f_w = float(n_tile_size) / m_n_vw * 2, f_h = float(n_tile_size) / m_n_vh * 2;
		for(int y = 0; y < m_n_vh; y += n_tile_size) {
			float f_y0 = float(y) / m_n_vh * 2 - 1;
			for(int x = 0; x < m_n_vw; x += n_tile_size, ++ n_first_tile_id) {
				float f_x0 = float(x) / m_n_vw * 2 - 1;
				m_shader.Bind(f_x0, f_y0, f_w, f_h, 0, 0, 1, 1);
				// set up the shader

				m_quad.Draw();
				// draw the quad (a waste, could draw all at once, using a repeating
				// texture; want to have it precise though. probably being paranoid)

				if(b_have_font) {
					char p_s_tile_name[256];
					stl_ut::Format(p_s_tile_name, sizeof(p_s_tile_name), "tile %05" _PRIsize, n_first_tile_id);

					float f_width = f_w * .75f;
					float f_zoom = f_width / m_font.v_Text_Size(p_s_tile_name, 1).x;
					// to center horizontally in the tile and to take 90%

					m_font.Generate_Geometry(font_vertex_list, (uint32_t)(font_vertex_list.size() / 5),
						font_index_list, p_s_tile_name, Vector2f(f_x0 + .5 * (f_w - f_width),
						-f_page_aspect * f_y0 - .5f * (f_page_aspect * f_h + (m_font.t_Font_Info().n_ascent -
						m_font.t_Font_Info().n_int_leading) * f_zoom)), f_zoom);
					// generate batched geometry for the tile names
				}
			}
		}
		// draw tiles

		printf("debug: the last tile was " PRIsize "\n", n_first_tile_id - 1);

		if(b_have_font) {
			Matrix4f I;
			I.Scaling(1, -1 / f_page_aspect, 1); // upside down and with aspect
			m_font_shader.Bind(I);
			//glActiveTexture(GL_TEXTURE0); // already did that above
			m_font.p_Texture_2D()->Bind();
			CGLElementArraySetup m_text_array(&font_vertex_list.front(), font_vertex_list.size() * sizeof(float),
				5 * sizeof(float), GL_FLOAT, 2, 3 * sizeof(float), GL_FLOAT, 3, 0, &font_index_list.front(),
				font_index_list.size(), GL_UNSIGNED_INT, m_font.n_Primitive_Mode());
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			m_text_array.Draw();
			glDisable(GL_BLEND);
			// render all the text in batch
		}
	}

	// todo - will need to split this to upload texture and to rendering tiles, mipmaps will be lazy evaluated,
	// will also need to specify the interior of the texture, a bit like bitblt does. this is needed when filling
	// multiple tiles with a texture. alternatively, this could be batched (give a list of dest subwindows and pads
	// and a corresponding list of source subwindows)

	// one would think there would be a need for one more version for the production code that will have also tile padding
	// tile padding is simple however, just extend both the destination coords and image coords
	// tile padding is *not* repeating the same border pixels around the tile, it is supposed to extend
	// it for filtering (and the filtering on the individual tiles is *not* clamp)

	void Draw(int n_x, int n_y, int n_width, int n_height,
		float f_s, float f_t, float f_tex_width, float f_tex_height, // note that there is no padding argument; padding is added by negative or larger-than-size texture coordinates ;)
		const TBmp *p_src, GLenum n_repeat_mode = GL_REPEAT, bool b_requires_upload = true) // b_requires_upload is error-prone // todo - implement a batched interface
	{
		float f_scale = (float(n_width) / (f_tex_width * p_src->n_width) + float(n_height) / (f_tex_height * p_src->n_height)) * .5f;
		float f_mip_level = log(f_scale) / log(2.0f);
		int n_max_levels = max(0, int(ceil(f_mip_level)));
		// mipmap reasoning

		if(b_requires_upload) { // if cleared then don't lose time by binding, it would presumedly be already bound
			glActiveTexture(GL_TEXTURE0); // ...
			m_texture.Bind();
			GLenum n_min_filter = (n_max_levels >= 1)? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR;
			if(m_n_last_min_filter != n_min_filter) {
				m_texture.Set_Min_Filter(n_min_filter);
				m_n_last_min_filter = n_min_filter;
			}
			glTexImage2D(m_texture.n_Target(), 0, m_n_internal_format, p_src->n_width,
				p_src->n_height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, p_src->p_buffer);
			glTexParameteri(m_texture.n_Target(), GL_TEXTURE_MAX_LEVEL, n_max_levels); // todo - time if this makes any difference
			if(n_max_levels >= 1)
				glGenerateMipmap(m_texture.n_Target());
			if(m_n_last_wrap_mode != n_repeat_mode) {
				m_texture.Set_Wrap_S(n_repeat_mode);
				m_texture.Set_Wrap_T(n_repeat_mode);
				m_n_last_wrap_mode = n_repeat_mode;
			}
		}
		// upload the texture

		float f_x0 = float(n_x) / m_n_vw * 2 - 1;
		float f_y0 = float(n_y) / m_n_vh * 2 - 1;
		float f_w = float(n_width) / m_n_vw * 2;
		float f_h = float(n_height) / m_n_vh * 2;

		/*float f_s0 = float(f_s);// / p_src->n_width;
		float f_t0 = float(f_t);// / p_src->n_height;
		float f_s = float(f_s + f_tex_width);// / p_src->n_width;
		float f_t = float(f_t + f_tex_height);// / p_src->n_height;*/

		m_shader.Bind(f_x0, f_y0, f_w, f_h, f_s, f_t, f_tex_width, f_tex_height);
		// set up the shader

		m_quad.Draw();
		// draw the quad
	}

	void Draw(int n_x, int n_y, int n_width, int n_height,
		const TBmp *p_src, int n_padding, GLenum n_repeat_mode = GL_REPEAT)
	{
		float f_scale = (float(n_width) / p_src->n_width + float(n_height) / p_src->n_height) * .5f;
		float f_mip_level = log(f_scale) / log(2.0f);
		int n_max_levels = max(0, int(ceil(f_mip_level)));

		glActiveTexture(GL_TEXTURE0); // ...
		m_texture.Bind();
		GLenum n_min_filter = (n_max_levels >= 1)? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR;
		if(m_n_last_min_filter != n_min_filter) {
			m_texture.Set_Min_Filter(n_min_filter);
			m_n_last_min_filter = n_min_filter;
		}
		glTexImage2D(m_texture.n_Target(), 0, m_n_internal_format, p_src->n_width,
			p_src->n_height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, p_src->p_buffer);
		glTexParameteri(m_texture.n_Target(), GL_TEXTURE_MAX_LEVEL, n_max_levels); // todo - time if this makes any difference
		if(n_max_levels >= 1)
			glGenerateMipmap(m_texture.n_Target());
		if(m_n_last_wrap_mode != n_repeat_mode) {
			m_texture.Set_Wrap_S(n_repeat_mode);
			m_texture.Set_Wrap_T(n_repeat_mode);
			m_n_last_wrap_mode = n_repeat_mode;
		}
		// upload the texture

		// todo - could do texture uploads using PBO and async mode, maybe even from multiple different threads
		// todo - could do batching of multiple texture uploads by setting up several textures for the upload

		float f_x0 = float(n_x - n_padding) / m_n_vw * 2 - 1;
		float f_y0 = float(n_y - n_padding) / m_n_vh * 2 - 1;
		float f_w = float(n_width + 2 * n_padding) / m_n_vw * 2;
		float f_h = float(n_height + 2 * n_padding) / m_n_vh * 2;

		float f_pad_x = float(n_padding) / n_width;
		float f_pad_y = float(n_padding) / n_height;
		float f_s0 = -f_pad_x;
		float f_t0 = -f_pad_y;
		float f_s = 1 + 2 * f_pad_x;
		float f_t = 1 + 2 * f_pad_y;

		m_shader.Bind(f_x0, f_y0, f_w, f_h, f_s0, f_t0, f_s, f_t);
		// set up the shader

		m_quad.Draw();
		// draw the quad
	}

	void Debug_ReadBack(TBmp *p_dest) const // note that a texture must be bound
	{
		glReadPixels(0, 0, p_dest->n_width, p_dest->n_height,
			GL_BGRA_EXT, GL_UNSIGNED_BYTE, p_dest->p_buffer);
	}

	bool Release()
	{
		m_fbo.Bind_ColorTexture(0, 0, 0);
		m_fbo.Release();
		glViewport(m_p_prev_wp[0], m_p_prev_wp[1], m_p_prev_wp[2], m_p_prev_wp[3]);
		return true;
	}
};

const float CGLTextureScalingUploader::p_quad[8] = {
	0, 0,
	0, 1,
	1, 0,
	1, 1
};
// t_odo - implement OpenGL texture rescaler to be able to upload the texture, make
//        (just enough) mipmaps and render it to a 2D texture / texture array layer

// t_odo - save the texture to see if it is correct

// t_odo - render transparent images in the cameras, to be able to see the projections

#define MEGATEXTURE_NEXT_FREE_TEXUNIT (GL_TEXTURE1)
// where to bind another texture

#define MEGATEXTURE_NEXT_FREE_TEXUNIT_INDEX ((MEGATEXTURE_NEXT_FREE_TEXUNIT) - GL_TEXTURE0)

#define DECLARE_MEGATEXTURE_UNIFORMS int n_megatexture_uniform;
// put this in your shader class (C++)

#define MEGATEXTURE_UNIFORMS_INITILAIZER \
	do { \
		n_megatexture_uniform = n_Get_Uniform_Location("n_megatexture_tex_unit"); \
		Uniform1i(n_megatexture_uniform, 0); \
	} while(0)
// put this at the end of your shader Compile() function (C++)

#define MEGATEXTURE_SAMPLE_FUNCTION_BODY \
	"uniform sampler2D n_megatexture_tex_unit;\n" \
	"vec4 megatexture2D(vec2 v_coord)\n" \
	"{\n" \
	"    return texture2D(n_megatexture_tex_unit, v_coord).bgra;\n" \
	"}\n"
// put this in your shader code (GLSL)

#define MEGATEXTURE_SAMPLE "megatexture2D"
// use this whenever sampling a texture (GLSL)
// this name will not change

// todo - get rid of macros and replace them by functions. now it is ok for rapid prototyping

class CGLMegatexture {
protected:
	// when sampling, one needs to divide by megatexture tile size to get tile id (better a POT)
	// then she needs to multiply the tile coordinates by tile scale (arbitrary integer)
	int m_n_tile_size; // tile size in the megatexture (not including the padding pixels, "physical" tile size is larger)
	int m_n_tile_padding; // tile padding in pixels (use 1 to get 1 more border pixel around the tile, so that they overlap and that filtering is cheap)
	int m_n_width; // total width of the megatexture (pixels)
	int m_n_height; // total height of the megatexture (pixels)
	int m_n_page_size; // all pages are square
	int m_n_page_num; // number of pages (layers in the array texture)
	int m_n_mipmap_level_num; // number of mip levels, including the base level

	uint64_t m_n_total_log_tiles; // number of total logical files in the base mip level

	size_t m_n_total_phys_tiles; // number of total physical tiles that can be allocated at once
	size_t m_n_next_free_phys_tile;

	typedef uint32_t TPhysTileId;
	typedef CLRU_Set<TPhysTileId> m_phys_tile_allocation; // this corresponds to physical tiles in texture

	// todo - write a function that profiles the size of all the stuff in memory, seems like there is a lot of structures

	enum {
		max_ResidentLODs_per_Tile = 4 // if there is more, start deleting them
	};

	struct TTileState {
		//uint32_t n_LODs_locked; // which of the LODs are locked and cannot be deleted (either for loading or because they are used in the current frame)
		// eh, this will cause problems, if one loads a texture and then not use it, it will remain locked until being used
		// could still theoretically keep a list of requests and "manually" unlock the ones that did not get used in building
		// the transfers for the current frame. also if rendering the feedback at half res, then the current frame fails
		// there are also problems with transparent objects (jitter + stipple somehow?)
		// add later maybe

		uint32_t n_LODs; // a bit-mask of allocated levels of detail (least significant bit corresponds to the base level, larger LOD is smaller level)
		TPhysTileId p_allocs[max_ResidentLODs_per_Tile]; // need to know where the LODs are actually allocated!

		int n_LOD_lock; // a LOD lock (locked to that and one lower level of detail; in case a better level of detail is available, can start fading)
		int n_fade_direction; // fade direction (+1 or -1 or 0 if not fading)
		double f_fade_time; // start of fading; if this is smaller than time - fade_duration then the fade is done

		TPhysTileId n_allocation; // id of the tile (x, y and layer)
	};
	typedef Vector2i TLogTileId;
	std::map<TLogTileId, TTileState> m_log_tile_allocation; // this corresponds to allocated logical tiles that map to possibly more than one physical tile

	std::vector<uint32_t> m_tile_indirection_table;
	CGLTexture_2D m_tile_indirection_texture;
	//CGLTexture_2D_Array m_phys_texture;
	// was initially thinking of trying an even-odd scheme where even layers have high lod
	// and odd layers one lower lod (all tiles allocated so that even and odd are above each other)
	// this wastes a lot of space in the low layers and makes it impossible to use one layer for
	// different levels of detail; this is all for the price of saving one texture sample from TIT
	// and also it makes the allocator simpler as it does not need to figure our how to stuff the tiles
	// of different sizes into a single page

	// idea: do the tiles of different LODs to be the *same size*, so that LOD_1 covers 2x the area of LOD_0
	// the padding is only at the edges so they take up the same space in the physical texture and the allocation
	// is easy. at the same time, any LOD_0 always mathes only a single LOD_N, N >= 0 (they are never misaligned)

	// to do a sample, we unfortunately need two TIT reads (can use lod clamp bits as weight clamp for either tile
	// and double the precision or store LOD in the high texel and 128-level clamp in the low texel)

	// the disadvantage is that producing low LOD tiles becomes costy (will really need the loader to downscale
	// the images, maybe try to produce the lower level tiles from the available higher level ones but that only solves
	// the problem when the LOD is decreasing)

	enum {
		tit_Pos_Bits = 8, // up to 256 x 256 tiles per page
		tit_Layer_Bits = 8, // up to 256 texture layers
		tit_LOD_Bits = 3, // covers 8 levels of detail
		tit_LODClamp_Bits = 8 - tit_LOD_Bits, // 32 values, 16 levels of LOD clamp (16 clamp from above and 16 from below)
		// don't change, this is hardcoded in the shader right now

		//   ccccclllxxxxxxxxyyyyyyyyllllllll
		// 32   27 24      16       8       0

		tit_LodClamp_Shift = 32 - tit_LODClamp_Bits,
		tit_LOD_Shift = tit_LodClamp_Shift - tit_LOD_Bits,
		// A

		tit_Pos_X_Shift = tit_LOD_Shift - tit_Pos_Bits,
		// R

		tit_Pos_Y_Shift = tit_Pos_X_Shift - tit_Pos_Bits,
		// G

		tit_Layer_Shift = tit_Pos_Y_Shift - tit_Layer_Bits
		// B
	};
	// texture indirection table texel division

	static uint32_t n_TIT_Encode(int n_tile_x, int n_tile_y, int n_tile_layer,
		int n_level_of_detail, float f_min_frac_lod, float f_max_frac_lod)
	{
		_ASSERTE(tit_Layer_Shift >= 0); // make sure there aren't more bits than can be mapped to RGBA8

		_ASSERTE(n_tile_x >= 0 && (unsigned int)n_tile_x <= n_Mask(uint32_t(tit_Pos_Bits)));
		_ASSERTE(n_tile_y >= 0 && (unsigned int)n_tile_y <= n_Mask(uint32_t(tit_Pos_Bits)));
		_ASSERTE(n_tile_layer >= 0 && (unsigned int)n_tile_layer <= n_Mask(uint32_t(tit_Layer_Bits)));
		_ASSERTE(n_level_of_detail >= 0 && (unsigned int)n_level_of_detail <= n_Mask(uint32_t(tit_LOD_Bits)));

		_ASSERTE(f_min_frac_lod >= 0);
		_ASSERTE(f_min_frac_lod <= f_max_frac_lod);
		_ASSERTE(f_max_frac_lod <= 1);
		_ASSERTE(f_min_frac_lod == 0 || f_max_frac_lod == 1); // only can clamp from one side, not from both simultaneously

		int n_lod_clamp = int(((f_min_frac_lod == 0)? .5f + .5f * f_max_frac_lod :
			.5f * f_min_frac_lod) * n_Mask(uint32_t(tit_LODClamp_Bits)));
		// quantize LOD clamp

		uint32_t n_argb = (n_tile_x << tit_Pos_X_Shift) | (n_tile_y << tit_Pos_Y_Shift) |
			(n_tile_layer << tit_Layer_Shift) | (n_level_of_detail << tit_LOD_Shift) |
			(n_lod_clamp << tit_LodClamp_Shift);
	}

	TPhysTileId n_TileIndex_to_PhysTileId(size_t n_phys_tile_index) const
	{
		int n_tiles = m_n_page_size / n_TileSize_Phys(); // tiles per page edge (round down)
		int n_tile_x = n_phys_tile_index % n_tiles;
		n_phys_tile_index /= m_n_page_size;
		int n_tile_y = n_phys_tile_index % n_tiles;
		n_phys_tile_index /= m_n_page_size;
		_ASSERTE(n_phys_tile_index <= INT_MAX); // by now it should be quite small
		int n_tile_layer = int(n_phys_tile_index);

		return (TPhysTileId)n_phys_tile_index; // unexpected :) // todo - figure out what function I really needed?
	}

public:
	CGLMegatexture(int n_width, int n_height, int n_page_size = 4096,
		int n_page_num = 8, int n_tile_size = 128, int n_tile_padding = 0) // 32 x 4k x 4k x RGBA = 2 GB (without mipmaps; with is 2.666 GB)
		:m_n_tile_size(n_tile_size), m_n_tile_padding(n_tile_padding), m_n_width(n_width),
		m_n_height(n_height), m_n_page_size(n_page_size), m_n_page_num(n_page_num),
		m_tile_indirection_texture((n_width + n_tile_size - 1) / n_tile_size,
		(n_height + n_tile_size - 1) / n_tile_size, GL_RGBA, false)
	{
		_ASSERTE(n_page_num <= (1U << tit_Layer_Bits)); // make sure we're not exceeding max layers
		_ASSERTE(n_page_size / (1U << tit_Pos_Bits) < (unsigned int)m_n_tile_size); // make sure the tiles are large enough to be able to address the pages

		unsigned int n_tiles = n_page_size / n_TileSize_Phys(); // tiles per page edge (round down)
		_ASSERTE(n_tiles > 1); // otherwise what's the point
		_ASSERTE(n_tiles < UINT32_MAX / n_tiles && n_tiles * n_tiles < UINT32_MAX / n_page_num); // make sure that tile ids can fit 32-bit integers

		m_n_total_phys_tiles = n_tiles * n_tiles * n_page_num;
		// number of allocable tiles; if we choose to use mipmapped layered texture then
		// we can also use the mipmapped dimensions in an even-odd manner (todo)

		m_n_next_free_phys_tile = 1;
		// will start assigning tiles from id 1

		m_n_total_log_tiles = uint64_t((n_width + n_TileSize_Log() - 1) / n_TileSize_Log()) *
			uint64_t((n_height + n_TileSize_Log() - 1) / n_TileSize_Log());

		m_tile_indirection_texture.Bind();
		m_tile_indirection_texture.Set_Mag_Filter(GL_NEAREST);
		m_tile_indirection_texture.Set_Min_Filter(GL_NEAREST);
		// indirection texture has nearest neighbor filter (is used to get info about the tiles)

		m_tile_indirection_table.resize(m_tile_indirection_texture.n_Width() *
			m_tile_indirection_texture.n_Height(), n_TIT_Encode(0, 0, 0, 0, 0, 0));
		// m_tile_indirection_table is a shadow copy of the tile indirection texture
		// initialy the entire TIT is pointing to tile zero which is a privileged tile
		// and will not be allocated for anyone else

		glTexSubImage2D(m_tile_indirection_texture.n_Target(), 0, 0, 0, m_tile_indirection_texture.n_Width(),
			m_tile_indirection_texture.n_Height(), GL_BGRA_EXT, GL_UNSIGNED_BYTE, &m_tile_indirection_table.front());
		// refresh TIT
	}

	bool Get_Tile(int n_tile_x, int n_tile_y)
	{
	}

	int n_TileSize_Log() const // power of two, logical size in megatexture coordinates
	{
		return m_n_tile_size;
	}

	int n_TileSize_Phys() const // non-power of two, larger, physical size in 2D array texture coordinates
	{
		return m_n_tile_size + 2 * m_n_tile_padding;
	}
};
// todo - implement simple static megatexture which just stuffs a large texture
//        atlas and fills the TIT

// todo - implement feedback rendering

/**
 *	@brief virtual texture atlas; arranges 2D images into a single texture atlas
 *		page (of theoretically unlimited or at least very large size)
 *
 *	This is different from the "classic" texture atlases in that the texture page
 *	is only a singe one and it is finally not stored in a texture but in a virtual
 *	texture with its own paging mechanism and practically unlimited size.
 */
class CVirtualTextureAtlas {
public:
	typedef TBinPackingItem<unsigned int> TImageRectangle;
	typedef CBinPackingItemManipulator<unsigned int, TImageRectangle> TImageHandler;
	typedef CBinPackingPage<unsigned int, TImageRectangle, TImageHandler> TAtlasPage;

protected:
	struct TPageFactory {
		unsigned int n_page_size;

		TPageFactory(unsigned int _n_page_size)
			:n_page_size(_n_page_size)
		{}

		TAtlasPage operator ()()
		{
			TAtlasPage p(n_page_size, n_page_size, max(32U, n_page_size / 32));
			n_page_size = 1; // only make a single page, not more
			return p;
		}
	};

	class CApplyScalePad {
	protected:
		double m_f_scale;
		unsigned int m_n_two_pad;

	public:
		CApplyScalePad(double f_scale, unsigned int n_pad)
			:m_f_scale(f_scale), m_n_two_pad(n_pad + n_pad)
		{}

		TImageRectangle operator ()(TImageRectangle t_rect) const // copy intended
		{
			t_rect.n_width = max(1U, (unsigned int)(ceil(t_rect.n_width * m_f_scale))) + m_n_two_pad;
			t_rect.n_height = max(1U, (unsigned int)(ceil(t_rect.n_height * m_f_scale))) + m_n_two_pad;
			return t_rect;
		}
	};

	class CUnApplyPad {
	protected:
		unsigned int m_n_pad;

	public:
		CUnApplyPad(unsigned int n_pad)
			:m_n_pad(n_pad)
		{}

		TImageRectangle operator ()(TImageRectangle t_rect) const // copy intended
		{
			t_rect.n_x += m_n_pad;
			t_rect.n_y += m_n_pad;
			t_rect.n_width -= 2 * m_n_pad;
			t_rect.n_height -= 2 * m_n_pad;
			return t_rect;
		}
	};

	/**
	 *	@brief ordering for the packed items
	 *
	 *	This groups items by orientation (tall items first since we're packing
	 *	vertically; the wide items might then be lucky to fit in the gaps above
	 *	and below) and sorts by decreasing size. This improves packing efficiency
	 *	by about 10% compared to random ordering.
	 */
	class CItemPackOrdering {
	public:
		bool operator ()(const TImageRectangle &r_t_a, const TImageRectangle &r_t_b) const
		{
			bool b_tall_a = r_t_a.n_width < r_t_a.n_height;
			bool b_tall_b = r_t_b.n_width < r_t_b.n_height;
			return (b_tall_a && !b_tall_b) || (!(!b_tall_a && b_tall_b) &&
				size_t(r_t_a.n_width) * r_t_a.n_height > size_t(r_t_b.n_width) * r_t_b.n_height);
		}
	};

	std::vector<TImageRectangle> m_texture_list;
	unsigned int m_n_padding;
	double m_f_scale;
	unsigned int m_n_bbox_width, m_n_bbox_height;

public:
	CVirtualTextureAtlas(unsigned int n_padding = 0)
		:m_n_padding(n_padding), m_f_scale(1), m_n_bbox_width(-1), m_n_bbox_height(-1)
	{}

	size_t n_ImageRect_Num() const
	{
		return m_texture_list.size();
	}

	const TImageRectangle &r_ImageRect(size_t n_index)
	{
		return m_texture_list[n_index];
	}

	unsigned int n_Image_Padding() const
	{
		return m_n_padding;
	}

	unsigned int n_Atlas_Width() const
	{
		return m_n_bbox_width;
	}

	unsigned int n_Atlas_Height() const
	{
		return m_n_bbox_height;
	}

	size_t n_Add_Image(unsigned int n_width, unsigned int n_height) // returns image id or -1 on failure
	{
		TImageRectangle t_rect(n_width, n_height);
		size_t n_id = m_texture_list.size();
		return (stl_ut::Resize_Add_1More(m_texture_list, t_rect))? n_id : size_t(-1);
	}

	bool Solve_BinPacking(unsigned int n_max_atlas_size,
		double f_min_scale = 1e-3, double f_scale_step = .9)
	{
		if(m_texture_list.empty()) {
			m_n_bbox_width = m_n_padding;
			m_n_bbox_height = m_n_padding;
			m_f_scale = 1;
			return true; // another job well done
		}

		std::vector<size_t> ordering(m_texture_list.size());
		stl_ut::IOTA(ordering.begin(), ordering.end());
		std::stable_sort(ordering.begin(), ordering.end(),
			stl_ut::CCompare_Indirect<TImageRectangle,
			CItemPackOrdering>(&m_texture_list.front(), m_texture_list.size()));
		// create ordering for item packing

		std::vector<TImageRectangle> ordered_rectangles(m_texture_list.size());
		for(size_t i = 0, n = m_texture_list.size(); i < n; ++ i)
			ordered_rectangles[i] = m_texture_list[ordering[i]];
		// reorder

		std::swap(m_texture_list, ordered_rectangles);
		bool b_result = _Solve_BinPacking(n_max_atlas_size, f_min_scale, f_scale_step); // if this throws then m_texture_list will not correspond to the original images
		std::swap(m_texture_list, ordered_rectangles);
		// swap and solve

		for(size_t i = 0, n = m_texture_list.size(); i < n; ++ i)
			m_texture_list[ordering[i]] = ordered_rectangles[i];
		// reorder back

		return b_result;
	}

protected:
	bool _Solve_BinPacking(unsigned int n_max_atlas_size,
		double f_min_scale = 1e-3, double f_scale_step = .9)
	{
		_ASSERTE(f_scale_step < .99 && f_scale_step > 0); // scale step must be less than one and positive at the same time
		_ASSERTE(f_min_scale > 0 && f_min_scale <= 1); // min scale must be positive and can be one (scaling not allowed)

		// t_odo - could sort the rectangles using indirect sort (the one with a permutation) and then order them back

		m_n_bbox_width = -1;
		m_n_bbox_height = -1;

		uint64_t n_area = 0;
		unsigned int n_max_size = 0;
		for(size_t i = 0, n = m_texture_list.size(); i < n; ++ i) {
			n_area += uint64_t(m_texture_list[i].n_width + 2 * m_n_padding) *
				(m_texture_list[i].n_height + 2 * m_n_padding);
			n_max_size = max(n_max_size, max(m_texture_list[i].n_width, m_texture_list[i].n_height));
		}
		n_max_size += 2 * m_n_padding;
		// get image stats

		double f_scale = 1;
		if(n_area / n_max_atlas_size >= n_max_atlas_size)
			f_scale = 1 / sqrt((double(n_area) / n_max_atlas_size) / n_max_atlas_size);
		// handle too many items

		if(n_max_size * f_scale > n_max_atlas_size)
			f_scale = double(n_max_atlas_size) / n_max_size;
		// handle too big items

		const size_t n_smallest_square = n_Make_Lower_POT(size_t(sqrt(double(n_area)) + 1));
		_ASSERTE(n_smallest_square <= UINT_MAX);
		// see what is the smallest square where the items would fit

		std::vector<TImageRectangle> scaled_list;
		for(; f_scale > f_min_scale; f_scale = (f_scale > f_min_scale && f_scale *
		   f_scale_step < f_min_scale)? f_min_scale : f_scale * f_scale_step) {
			if(f_scale < 1) {   
				if(!stl_ut::Resize_To_N(scaled_list, m_texture_list.size()))
					return false;
				std::transform(m_texture_list.begin(), m_texture_list.end(),
					scaled_list.begin(), CApplyScalePad(f_scale, m_n_padding));
			}
			std::vector<TImageRectangle> &r_items_to_pack = (f_scale < 1)? scaled_list : m_texture_list;
			// rescale the items

			if(f_scale == 1) { // waste of time trying lower resolutions if the scale is less than 1; did not fit before
				for(unsigned int n_page_size = min(n_max_atlas_size, max(1024U, (unsigned int)(n_smallest_square *
				   f_scale))); n_page_size <= n_max_atlas_size; n_page_size += n_page_size) {
#ifdef _DEBUG
					printf("debug: trying to fit the textures in a single %u x %u px page\n", n_page_size, n_page_size);
#endif // _DEBUG
					TPageFactory page_factory(n_page_size);
					std::list<TAtlasPage> page_list;
					if(CBinPackingSolver<unsigned int, TImageRectangle, TImageHandler>::First_Fit(page_list,
					   r_items_to_pack, page_factory, false) && page_list.size() == 1) {
						TAtlasPage &p = page_list.front();
						p.Set_Size(p.n_BoundingBox_Width(), p.n_BoundingBox_Height());
						// shrink

						m_n_bbox_width = p.n_Width();
						m_n_bbox_height = p.n_Height();
						// remember

						uint64_t n_mem = n_Align_Up(m_n_bbox_width, 64U) *
							uint64_t(n_Align_Up(m_n_bbox_height, 64U)) * 4;
						// size of RGBA8 image

						printf("debug: packed all items, at scale %g (%d x %d px, " PRIsizeB
							"B without mipmaps, %.2f%% efficient)\n", f_scale, p.n_Width(),
							p.n_Height(), PRIsizeBparams(n_mem), p.f_Efficiency() * 100);
						std::transform(r_items_to_pack.begin(), r_items_to_pack.end(),
							r_items_to_pack.begin(), CUnApplyPad(m_n_padding));
						page_list.front().Debug_Raster("texture_atlas.tga");
						// debug

						if(&r_items_to_pack != &m_texture_list)
							std::swap(r_items_to_pack, m_texture_list); // swap now, not earlier
						return true;
					}
				}
			}
			if(f_scale < 1 || (n_max_atlas_size > 1024 && !b_Is_POT(n_max_atlas_size))) {
				const unsigned int n_page_size = n_max_atlas_size;
#ifdef _DEBUG
				printf("debug: trying to fit the textures in a single %u x %u px page at scale %f\n",
					n_page_size, n_page_size, f_scale);
#endif // _DEBUG
				TPageFactory page_factory(n_page_size);
				std::list<TAtlasPage> page_list;
				if(CBinPackingSolver<unsigned int, TImageRectangle, TImageHandler>::First_Fit(page_list,
				   r_items_to_pack, page_factory, false) && page_list.size() == 1) {
					TAtlasPage &p = page_list.front();
					p.Set_Size(p.n_BoundingBox_Width(), p.n_BoundingBox_Height());
					// shrink

					m_n_bbox_width = p.n_Width();
					m_n_bbox_height = p.n_Height();
					// remember

					uint64_t n_mem = n_Align_Up(m_n_bbox_width, 64U) *
						uint64_t(n_Align_Up(m_n_bbox_height, 64U)) * 4;
					// size of RGBA8 image

					printf("debug: packed all items, at scale %g (%d x %d px, " PRIsizeB
						"B without mipmaps, %.2f%% efficient)\n", f_scale, p.n_Width(),
						p.n_Height(), PRIsizeBparams(n_mem), p.f_Efficiency() * 100);
					std::transform(r_items_to_pack.begin(), r_items_to_pack.end(),
						r_items_to_pack.begin(), CUnApplyPad(m_n_padding));
					page_list.front().Debug_Raster("texture_atlas.tga");
					// debug

					if(&r_items_to_pack != &m_texture_list)
						std::swap(r_items_to_pack, m_texture_list); // swap now, not earlier
					return true;
				}
			}
			// try to fit everything into a single page
		}

		printf("debug: failed to pack, last scale was %g\n", f_scale);

		return false; // could not fit
	}
};

#endif // __MEGATEXTURE_V0_INCLUDED
