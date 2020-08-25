/*
								+--------------------------------+
								|                                |
								|***  Overlapped file reader  ***|
								|                                |
								|  Copyright © -tHE SWINe- 2016  |
								|                                |
								|            FastIO.h            |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __OVERLAPPED_FILE_READER_INCLUDED
#define __OVERLAPPED_FILE_READER_INCLUDED

/**
 *	@file FastIO.h
 *	@brief file readers running in a second thread for increased performance
 *	@author -tHE SWINe-
 *	@date 2016
 */

class CFileReader : public CRunable_Thread_ShallowCopy { // not faster than thread-less (~4 MB/s) // bad design - would have to read blocks of a couple MB of data so that the mutexes pay off, and do the line parsing in ReadLine(), maybe someday ...
protected:
	FILE *m_p_fr;
	CProducerConsumerQueue<std::string*> m_queue;
	std::vector<std::string> m_item_storage; // must be here, if thread local then it gets deleted upon EOF, before all is read out
	bool m_b_bad_alloc, m_b_error, m_b_stop;
	bool m_b_skip_empty_lines;

public:
	CFileReader(FILE *p_fr, bool b_skip_empty_lines, size_t n_queue_length = 1024)
		:m_p_fr(p_fr), m_queue(n_queue_length), m_b_bad_alloc(false), m_b_error(false), m_b_stop(false),
		m_b_skip_empty_lines(b_skip_empty_lines), m_item_storage(n_queue_length + 2)
	{
		Start();
	}

	~CFileReader()
	{
		WaitForFinish();
		fclose(m_p_fr);
	}

	bool WaitForFinish()
	{
		m_b_stop = true;
		return this->CRunable_Thread_ShallowCopy::WaitForFinish();
	}

	bool b_EOF()
	{
		return m_queue.b_Finished();
	}

	bool ReadLine(std::string &r_s_line)
	{
		std::string *p_line;
		bool b_result = m_queue.Get(p_line) && !m_b_error && !m_b_bad_alloc;
		if(b_result && p_line)
			r_s_line.swap(*p_line); // swap - not copy, the buffers will be recycled in case r_s_line is not deleted
		return b_result;
	}

protected:
	virtual void Run()
	{
		try {
			const size_t n_list_num = m_item_storage.size();
			size_t n_next_free_list = 0;

			while(!feof(m_p_fr) && !m_b_stop) {
				std::string &s_line = m_item_storage[n_next_free_list];
				if(!stl_ut::ReadLine(s_line, m_p_fr)) {
					m_b_error = true;
					break;
				}
				if(m_b_skip_empty_lines) {
					stl_ut::TrimSpace(s_line);
					if(s_line.empty())
						continue;
				}
				m_queue.Put(&s_line);
				n_next_free_list = (n_next_free_list + 1) % n_list_num;
			}
		} catch(std::bad_alloc&) {
			m_b_bad_alloc = true;
		}
		m_queue.Signal_Finished(); // done
	}

	CFileReader(const CFileReader &r_other); // no-copy
	CFileReader &operator =(const CFileReader &r_other); // no-copy
};

class CNumFileReader : public CRunable_Thread_ShallowCopy { // not faster than thread-less (~2.75 MB/s)
protected:
	FILE *m_p_fr;
	CProducerConsumerQueue<const std::vector<float>*> m_queue;
	bool m_b_bad_alloc, m_b_error, m_b_stop;
	bool m_b_skip_empty_lines;

	std::vector<std::vector<float> > m_item_storage; // must be here, if thread local then it gets deleted upon EOF, before all is read out

public:
	CNumFileReader(FILE *p_fr, bool b_skip_empty_lines, size_t n_queue_length = 1024)
		:m_p_fr(p_fr), m_queue(n_queue_length), m_b_bad_alloc(false), m_b_error(false), m_b_stop(false),
		m_b_skip_empty_lines(b_skip_empty_lines), m_item_storage(n_queue_length + 2) // two more so that there is always at least one which is not queued (assuming the caller dequeues only one at a time)
	{
		Start();
	}

	~CNumFileReader()
	{
		WaitForFinish();
		fclose(m_p_fr);
	}

	bool WaitForFinish()
	{
		m_b_stop = true;
		return this->CRunable_Thread_ShallowCopy::WaitForFinish();
	}

	bool b_EOF()
	{
		return m_queue.b_Finished();
	}

	const std::vector<float> *ReadNumLine()
	{
		const std::vector<float> *p_list = 0;
		bool b_result = m_queue.Get(p_list) && !m_b_error && !m_b_bad_alloc;
		return (b_result)? p_list : 0;
	}

protected:
	virtual void Run() // for some reason this is buggy
	{
		try {
			const size_t n_list_num = m_item_storage.size();
			size_t n_next_free_list = 0;

			std::vector<std::string> field_list;
			std::string s_line;

			while(!feof(m_p_fr) && !m_b_stop) {
				if(!stl_ut::ReadLine(s_line, m_p_fr)) {
					m_b_error = true;
					break;
				}
				if(m_b_skip_empty_lines) {
					stl_ut::TrimSpace(s_line);
					if(s_line.empty())
						continue;
				}
				if(!stl_ut::Split(field_list, s_line, " ", 0)) {
					m_b_bad_alloc = true;
					break;
				}
				// read a line, split it to fields

				std::vector<float> &num_list = m_item_storage[n_next_free_list];
				// get a free list

				num_list.resize(field_list.size());
				std::transform(field_list.begin(), field_list.end(), num_list.begin(), TConv());
				// parse the numbers

				if(!m_queue.Put(&num_list)) {
					m_b_error = true;
					break;
				}
				// put it in the queue (only m_queue.n_Size() items in the
				// queue at a time + 1 being used by each consumer)

				n_next_free_list = (n_next_free_list + 1) % n_list_num;
				// advance to the next free item
			}
		} catch(std::bad_alloc&) {
			m_b_bad_alloc = true;
		}
		m_queue.Signal_Finished(); // done
	}

	struct TConv {
		double operator ()(const std::string &r_s_str)
		{
			return atof(r_s_str.c_str());
		}
	};

	CNumFileReader(const CNumFileReader &r_other); // no-copy
	CNumFileReader &operator =(const CNumFileReader &r_other); // no-copy
};

#endif // !__OVERLAPPED_FILE_READER_INCLUDED
