/*
								+--------------------------------+
								|                                |
								| ***  Universal file lists  *** |
								|                                |
								|  Copyright © -tHE SWINe- 2016  |
								|                                |
								|           FileList.h           |
								|                                |
								+--------------------------------+
*/

#pragma once
#ifndef __UNIVERSAL_FILE_LIST_INCLUDED
#define __UNIVERSAL_FILE_LIST_INCLUDED

/**
 *	@file FileList.h
 *	@brief simple universal file lists
 *	@author -tHE SWINe-
 *	@date 2016
 */

bool b_Match_Wildcard(const char *p_s_pattern, const char *p_s_query, bool b_case_sensitive = true)
{
	_ASSERTE(p_s_pattern && p_s_query);
	while(*p_s_pattern && *p_s_query) {
		if(*p_s_pattern == '*') { // a wildcard; need to match the next character
			char n_next_char = *(++ p_s_pattern);
			while(n_next_char == '*')
				n_next_char = *(++ p_s_pattern); // eat runs of consecutive '*' characters, they behave as one
			if(!n_next_char)
				return true; // a wildcard till the end, match everything
			for(; *p_s_query; ++ p_s_query) { // loop till the end
				if((b_case_sensitive && n_next_char == *p_s_query) ||
				   (!b_case_sensitive && tolower(uint8_t(n_next_char)) == tolower(uint8_t(*p_s_query)))) { // or do case insensitive match here
					if(b_Match_Wildcard(p_s_pattern + 1, p_s_query + 1, b_case_sensitive))
						return true;
					// indeterministically match possible places where the next regular character of the pattern could be
				}
			}
			return false; // could not match the rest of the pattern
		} else if((b_case_sensitive && *p_s_pattern != *p_s_query) ||
		   (!b_case_sensitive && tolower(uint8_t(*p_s_pattern)) != tolower(uint8_t(*p_s_query)))) // or do case insensitive match here
			return false; // a mismatch
		else {
			++ p_s_pattern;
			++ p_s_query; // advance
		}
	}
	return !*p_s_pattern && !*p_s_query; // only matches if both at the end

	// note that this uses implicit stack; should rewrite to explicit.

	// note that this might attempt to rematch the same suffixes many times,
	// could cache the results if very long patterns were expected.

	// note that the cost is O(n^k), where n is length of the query and k is
	// the number of (non-consecutive) asterisks in the pattern (probably ok
	// for filenames but horrible in theory).
}

class CFilterFileListener {
protected:
	std::vector<std::string> &m_r_file_list;
	const char *m_p_s_pattern;

public:
	CFilterFileListener(std::vector<std::string> &r_file_list, const char *p_s_pattern)
		:m_r_file_list(r_file_list), m_p_s_pattern(p_s_pattern)
	{}

	bool operator ()(const TFileInfo &r_t_file) // throw(std::bad_alloc)
	{
		if(r_t_file.b_directory)
			return true;
		// skip dirs

		if(b_Match_Wildcard(m_p_s_pattern, r_t_file.p_s_FileName(), CPath::case_Sensitive))
			m_r_file_list.push_back(r_t_file.s_filename);
		// match and remember

		return true;
	}
};

bool Resolve_FileList(std::vector<std::string> &r_file_list, const char *p_s_files_spec)
{
	r_file_list.clear();
	if(!p_s_files_spec)
		return true; // no images at all
	if(strchr(p_s_files_spec, '#')) { // a printf pattern
		std::string s_image_pattern = p_s_files_spec; // throws
		if(!CGLThreadedFrameWriter::Make_Pattern(s_image_pattern)) {
			fprintf(stderr, "error: couldnt make image pattern: \'%s\'\n", p_s_files_spec);
			return false;
		}
		std::string s_image;
		for(size_t i = 0, n_fail_num = 0; n_fail_num < 100; ++ i) {
			if(!stl_ut::Format(s_image, s_image_pattern.c_str(), i))
				return false;
			if(CPath::b_Exists(s_image)) { // is that a file
				r_file_list.push_back(s_image); // throws
				n_fail_num = 0; // reset that
			} else
				++ n_fail_num;
		}
		// enumerate
	} else if(strchr(p_s_files_spec, '*')) { // a wildcard
		std::string s_path = p_s_files_spec, s_pattern; // throws
		if(!CPath::Split(s_path, s_pattern))
			return false;
		CFilterFileListener listener(r_file_list, s_pattern.c_str());
		if(!CDirTraversal::Traverse2(s_path.c_str(), listener, false))
			return false;
	} else if(CPath::b_Exists(p_s_files_spec)) { // throws // a text file with the list of images
		FILE *p_fr;
#if defined(_MSC_VER) && !defined(__MWERKS__) && _MSC_VER >= 1400
		if(fopen_s(&p_fr, p_s_files_spec, "r"))
#else //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
		if(!(p_fr = fopen(p_s_files_spec, "r")))
#endif //_MSC_VER && !__MWERKS__ && _MSC_VER >= 1400
			return false;
		std::string s_path = p_s_files_spec, s_line; // throws
		if(!CPath::Split(s_path, s_line))
			return false;
		while(!feof(p_fr)) {
			if(!stl_ut::ReadLine(s_line, p_fr)) {
				fclose(p_fr);
				return false;
			}
			stl_ut::TrimSpace(s_line);
			if(s_line.empty())
				continue;
			// read a single line

			if(!CPath::b_Is_Absolute(s_line))
				CPath::Join(s_line, s_path, s_line); // the paths are relative to the list file
			if(CPath::b_Exists(s_line)) // is that a file
				r_file_list.push_back(s_line); // throws
			else
				fprintf(stderr, "warning: skipping \'%s\': not a file\n", s_line.c_str());
		}
		fclose(p_fr);
		// read an image list
	} else {
		fprintf(stderr, "error: failed to interpret image list; use \"numeric_####.png\", "
			"\"wildcard*.png\" or \"image_list.txt\"\n");
		return false;
	}
	// get image list

	return true;
}

#endif // !__UNIVERSAL_FILE_LIST_INCLUDED
