/*
								+--------------------------------+
								|                                |
								|     ***  LRU item set  ***     |
								|                                |
								|  Copyright © -tHE SWINe- 2016  |
								|                                |
								|             LRU.h              |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __LEAST_RECENTLY_USED_SET_INCLUDED
#define __LEAST_RECENTLY_USED_SET_INCLUDED

/**
 *	@file LRU.h
 *	@brief LRU item set
 *	@author -tHE SWINe-
 *	@date 2016
 */

#include <algorithm>
#include <functional>
#include <list>
#if defined(_MSC_VER) && !defined(__MWERKS__)
#include <unordered_map>
#else // _MSC_VER && !__MWERKS__
#include <tr1/unordered_map>
#endif // _MSC_VER && !__MWERKS__

template <class CKey, class CHasher = std::tr1::hash<CKey> >
class CLRU_Set {
public:
	typedef CKey _TyKey;
	typedef std::list<_TyKey> CKeyList;
	typedef typename CKeyList::iterator CIterator;
	typedef typename CKeyList::const_iterator CConstIterator;

	typedef std::tr1::unordered_map<_TyKey, CIterator, CHasher> CKeyMap;

protected:
	CKeyList m_key_list; // this is ordered and can be swapped quickly, iterators not invalidated
	CKeyMap m_key_lookup; // provides random access to the list

public:
	~CLRU_Set()
	{
		Check();
	}

	bool b_Empty() const
	{
		_ASSERTE(m_key_list.size() == m_key_lookup.size());
		return m_key_list.empty();
	}

	size_t n_Size() const
	{
		_ASSERTE(m_key_list.size() == m_key_lookup.size());
		return m_key_list.size();
	}

	void Clear()
	{
		Check();
		m_key_list.clear();
		m_key_lookup.clear();
	}

	CConstIterator p_Begin_It() const
	{
		return m_key_list.begin();
	}

	CConstIterator p_End_It() const
	{
		return m_key_list.end();
	}

	bool b_Contains(_TyKey k) const
	{
		return m_key_lookup.count(k);
	}

	// returns true on success, false on failure (new key not inserted, otherwise the set remains in a consistent state)
	bool Insert(_TyKey k) // throw()
	{
		typename CKeyMap::iterator p_map_it = m_key_lookup.find(k);
		if(p_map_it != m_key_lookup.end()) {
			_ASSERTE(!m_key_list.empty());
			if((*p_map_it).second != m_key_list.begin()) {
				m_key_list.splice(m_key_list.begin(), m_key_list, (*p_map_it).second); // move to the beginning, no allocs are made (guaranteed
				//m_key_list.erase((*p_map_it).second); // erase from the current position (the rest of the iterators is ok)
				//(*p_map_it).second = m_key_list.push_front(k); // inser at the beginning
				// this generally should not throw if the list conserves memory
			}
		} else {
			try {
				p_map_it = m_key_lookup.insert(std::make_pair(k, m_key_list.end())).first;
				m_key_list.push_front(k); // add a new key
				(*p_map_it).second/*m_key_lookup[k]*/ = m_key_list.begin();
				// this throws
			} catch(std::bad_alloc&) {
				if(p_map_it != m_key_lookup.end())
					m_key_lookup.erase(p_map_it);
				// remove from the map

				_ASSERTE(std::find(m_key_list.begin(), m_key_list.end(), k) == m_key_list.end());
				// cannot be present in the list, otherwise it would not throw

				Check();
				// debug the set

				return false;
			}
		}

		Check();

		return true;
	}

	void Erase(_TyKey k) // erases a specific key
	{
		typename CKeyMap::iterator p_map_it = m_key_lookup.find(k);
		_ASSERTE(p_map_it != m_key_lookup.end()); // must exist
		m_key_list.erase((*p_map_it).second);
		m_key_lookup.erase(p_map_it);

		Check();
	}

	void Erase_Back(size_t n) // erases the last n keys
	{
		_ASSERTE(n <= m_key_list.size());
		CIterator p_key_it;
		for(p_key_it = m_key_list.end(); n > 0; -- n) {
			-- p_key_it; // here

			m_key_lookup.erase(*p_key_it);
			// drop the iterators from the map
		}
		m_key_list.erase(p_key_it, m_key_list.end());
		// erase the keys

		Check();
	}

protected:
	inline void Check()
	{
#ifdef _DEBUG
		_ASSERTE(m_key_list.size() == m_key_lookup.size());
		for(CKeyMap::iterator p_map_it = m_key_lookup.begin(), p_end_it =
		   m_key_lookup.end(); p_map_it != p_end_it; ++ p_map_it)
			_ASSERTE((*p_map_it).first == *((*p_map_it).second));
		// make sure each map entry points to the correct key in the list
#endif // _DEBUG
		// debug consistency check
	}
};

#endif // !__LEAST_RECENTLY_USED_SET_INCLUDED
