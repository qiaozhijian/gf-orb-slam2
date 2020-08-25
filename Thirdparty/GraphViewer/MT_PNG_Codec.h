
#pragma once

// todo - header, doc

// 2014-12-09 - improved error handling

#include "../UberLame_src/Thread.h"
#include "../UberLame_src/iface/PNGLoad.h"

class CMTEncoder {
protected:
	struct TWorkItem {
		TBmp *p_bitmap;
		std::string s_filename;
	};

	typedef CProducerConsumerQueue<TWorkItem> _TyQueue;

	friend class CWorker;
	class CWorker : public CRunable_Thread_ShallowCopy {
	protected:
		_TyQueue *m_p_queue;
		_TyQueue *m_p_return_queue;
		const CMTEncoder *m_p_parent;

	public:
		void Configure(_TyQueue &r_queue, _TyQueue &r_return_queue, const CMTEncoder *p_parent) // to provide default argless ctor
		{
			m_p_queue = &r_queue;
			m_p_return_queue = &r_return_queue;
			m_p_parent = p_parent;
		}

		virtual inline void Run() // throws(std::runtime_error)
		{
			TWorkItem t_work;
			for(;;) {
				if(!m_p_queue->Get(t_work)) {
					if(m_p_queue->b_Finished())
						return;
					//throw std::runtime_error("work queue Get() failed"); // exceptions won't be transported
					m_p_parent->RaiseError(false);
					return;
				}
				// get work

				if(m_p_parent->b_FlipBitmaps())
					t_work.p_bitmap->Flip(true);
				if(!CPngCodec::Save_PNG(t_work.s_filename.c_str(), *t_work.p_bitmap, m_p_parent->b_BGRA())) { // ARGB
					m_p_parent->RaiseError(true);
					return;
				}

				if(!m_p_return_queue->Put(t_work)) {
					m_p_parent->RaiseError(false);
					return;
				}
			}
		}
	};

protected:
	_TyQueue queue, return_queue;
	std::vector<CWorker> worker_list;
	std::vector<TBmp*> bitmap_list;
	bool m_b_flip_bitmaps, m_b_BGRA;
	mutable bool m_b_error, m_b_io_error;

public:
	CMTEncoder(int n_fbo_width, int n_fbo_height, bool b_flip = true, bool b_BGRA = true,
		size_t n_worker_num = max(size_t(2), CThread::n_CPU_Num()) - 1) // throw(std::bad_alloc, std::runtime_error)
		:queue(n_worker_num), return_queue(n_worker_num), worker_list(n_worker_num),
		bitmap_list(n_worker_num + 1), m_b_flip_bitmaps(b_flip), m_b_BGRA(b_BGRA),
		m_b_error(false), m_b_io_error(false)
	{
		_ASSERTE(n_worker_num > 0);
		_ASSERTE(n_worker_num < SIZE_MAX); // want to alloc one extra bitmap
		_ASSERTE(n_fbo_width > 0 && n_fbo_height > 0);

		for(size_t i = 0; i < n_worker_num + 1; ++ i) {
			TBmp *p_bitmap;
			if(!(p_bitmap = TBmp::p_Alloc(n_fbo_width, n_fbo_height)))
				throw std::bad_alloc(); // rethrow
			bitmap_list[i] = p_bitmap;
			// make a list of bitmaps

			if(i < n_worker_num) {
				TWorkItem t_empty;
				t_empty.p_bitmap = p_bitmap;
				if(!return_queue.Put(t_empty))
					throw std::runtime_error("queue Put() failed"); // actually no big deal, just a failure on mutex / semaphore operation, nothing besides the queue is damaged
				// put the bitmaps to the return queue
			}
		}
		// alloc bitmaps

		for(size_t i = 0; i < n_worker_num; ++ i) {
			worker_list[i].Configure(queue, return_queue, this);
			if(!worker_list[i].Start()) {
				if(queue.Signal_Finished()) {
					for(size_t j = 0; j < i; ++ j)
						worker_list[j].WaitForFinish();
				}
				// gracefully shut down the waiting threads

				throw std::runtime_error("thread Start() failed"); // also no big deal, just that there are some hanging threads, waiting for the queue
			}
			// start the worker
		}
		// init the workers
	}

	~CMTEncoder() // throw(std::runtime_error)
	{
		if(!Synchronize()) {
			#pragma warning(suppress: 4297)
			throw std::runtime_error("CMTEncoder::Synchronize() failed");
		}
	}

	TBmp *p_Get_EmptyBitmap()
	{
		_ASSERTE(!bitmap_list.empty());
		return (bitmap_list.empty())? 0 : bitmap_list.back(); // there is one extra bitmap
	}

	bool Enqueue_Bitmap(const char *p_s_filename, TBmp *p_bitmap)
	{
		_ASSERTE(p_bitmap == p_Get_EmptyBitmap());
		// must be the same

		if(m_b_error)
			return false;
		// in case an error occured, don't enqueue more work

		TWorkItem t_empty;
		if(!return_queue.Get(t_empty)) // get an empty bitmap
			return false; //throw std::runtime_error("queue Get() failed");
		// get an empty work-item (may block)

		_ASSERTE(t_empty.p_bitmap->n_width == p_bitmap->n_width &&
			t_empty.p_bitmap->n_height == p_bitmap->n_height);
		t_empty.p_bitmap->b_alpha = p_bitmap->b_alpha;
		t_empty.p_bitmap->b_grayscale = p_bitmap->b_grayscale;
		t_empty.p_bitmap->n_former_bpc = p_bitmap->n_former_bpc;
		std::swap(t_empty.p_bitmap->p_buffer, p_bitmap->p_buffer); // just swap the buffers
		// copy the bitmap information

		if(!stl_ut::AssignCStr(t_empty.s_filename, p_s_filename))
			return false;
		// copy the filename

		if(!queue.Put(t_empty)) // put it to be processed
			return false; //throw std::runtime_error("queue Put() failed");
		return true;
	}

	bool Synchronize()
	{
		if(queue.b_Finished())
			return true; // well done
		if(!queue.Signal_Finished())
			return false;
		bool b_result = true;
		for(size_t i = 0, n_worker_num = worker_list.size(); i < n_worker_num; ++ i) {
			if(!worker_list[i].WaitForFinish())
				b_result = false;
		}
		if(b_result) { // otherwise could cause access violations
			worker_list.clear();
			for(size_t i = 0, n = bitmap_list.size(); i < n; ++ i)
				bitmap_list[i]->Delete();
			bitmap_list.clear();
		}
		if(m_b_error) {
			if(m_b_io_error)
				throw std::runtime_error("I/O error");
			else
				throw std::runtime_error("threading error");
		}
		return b_result;
	}

	const bool b_BGRA() const
	{
		return m_b_BGRA;
	}

	const bool b_FlipBitmaps() const
	{
		return m_b_flip_bitmaps;
	}

private:
	void RaiseError(bool b_io_error) const
	{
		if(!m_b_error)
			m_b_error = true;
		if(!m_b_io_error && b_io_error)
			m_b_io_error = true;
		// all threads just write true, no synchronization needed
	}

	CMTEncoder(const CMTEncoder &r_other); // no-copy
	CMTEncoder &operator =(const CMTEncoder &r_other); // no-copy
};
