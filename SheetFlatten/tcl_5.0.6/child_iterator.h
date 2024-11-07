/*******************************************************************************
Tree Container Library: Generic container library to store data in tree-like structures.
Copyright (c) 2006  Mitchel Haas

This software is provided 'as-is', without any express or implied warranty. 
In no event will the author be held liable for any damages arising from 
the use of this software.

Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1.	The origin of this software must not be misrepresented; 
you must not claim that you wrote the original software. 
If you use this software in a product, an acknowledgment in the product 
documentation would be appreciated but is not required.

2.	Altered source versions must be plainly marked as such, 
and must not be misrepresented as being the original software.

3.	The above copyright notice and this permission notice may not be removed 
or altered from any source distribution.

For complete documentation on this library, see http://www.datasoftsolutions.net
Email questions, comments or suggestions to mhaas@datasoftsolutions.net
*******************************************************************************/
#pragma once
#include <set>
#include <stack>
#include <queue>
#include <algorithm>
#include <iterator>

namespace tcl 
{
	// forward declaration
	template<typename T, typename U, typename V> class basic_tree;
	template<typename T> class sequential_tree;
	template<typename T, typename U, typename V> class associative_tree;
	template<typename T, typename U, typename V> class associative_reverse_iterator;
	template<typename T, typename U, typename V> class sequential_reverse_iterator;
	template<typename T, typename U, typename V, typename W> class const_pre_order_descendant_iterator;
	template<typename T, typename U, typename V, typename W> class const_post_order_descendant_iterator;

	template<typename T, typename U, typename V> class const_associative_iterator;
	template<typename T, typename U, typename V> class associative_iterator;
	template<typename T, typename U, typename V> class const_sequential_iterator;
	template<typename T, typename U, typename V> class sequential_iterator;

	// comparison operators
	template<typename T, typename U, typename V> bool operator == (const const_associative_iterator<T, U, V>& lhs, const const_associative_iterator<T, U, V>& rhs) { return lhs.pParent == rhs.pParent && lhs.it == rhs.it; }
	template<typename T, typename U, typename V> bool operator != (const const_associative_iterator<T, U, V>& lhs, const const_associative_iterator<T, U, V>& rhs) { return !(lhs == rhs); }

	template<typename T, typename U, typename V> bool operator == (const const_sequential_iterator<T, U, V>& lhs, const const_sequential_iterator<T, U, V>& rhs ) { return lhs.pParent == rhs.pParent && lhs.it == rhs.it; }
	template<typename T, typename U, typename V> bool operator != (const const_sequential_iterator<T, U, V>& lhs, const const_sequential_iterator<T, U, V>& rhs ) { return !(lhs == rhs); }
	template<typename T, typename U, typename V> bool operator < (const const_sequential_iterator<T, U, V>& lhs, const const_sequential_iterator<T, U, V>& rhs) { return lhs.it < rhs.it && lhs.pParent == rhs.pParent; }
	template<typename T, typename U, typename V> bool operator <= (const const_sequential_iterator<T, U, V>& lhs, const const_sequential_iterator<T, U, V>& rhs) { return lhs < rhs || lhs == rhs; }
	template<typename T, typename U, typename V> bool operator > (const const_sequential_iterator<T, U, V>& lhs, const const_sequential_iterator<T, U, V>& rhs) { return !(lhs <= rhs); }
	template<typename T, typename U, typename V> bool operator >= (const const_sequential_iterator<T, U, V>& lhs, const const_sequential_iterator<T, U, V>& rhs) { return !(lhs < rhs); }

}


/************************************************************************/
/* associative tree child iterators                                     */
/************************************************************************/


template<typename stored_type, typename tree_type, typename container_type>
class tcl::const_associative_iterator : public std::iterator<std::bidirectional_iterator_tag, stored_type>
{
public:
	// constructors/destructor
	const_associative_iterator() : pParent(0) {}
	virtual ~const_associative_iterator() {}
protected:
	explicit const_associative_iterator(typename container_type::const_iterator iter, const associative_tree<stored_type, tree_type, container_type>* pCalled_node) : it(iter), pParent(pCalled_node) {}
	// copy constructor, and assignment operator will be compiler generated correctly

public:
	// overloaded operators
	const stored_type& operator*() const { return  *(*it)->get(); }
	const stored_type* operator->() const { return (*it)->get(); }

	const_associative_iterator& operator ++() { ++it; return *this; }
	const_associative_iterator operator ++(int) { const_associative_iterator old(*this); ++*this; return old; }
	const_associative_iterator& operator --() { --it; return *this; }
	const_associative_iterator operator --(int) { const_associative_iterator old(*this); --*this; return old; }

	// public interface
	const tree_type* node() const { return *it; }

	// data
protected:
	typename container_type::const_iterator it;
	const associative_tree<stored_type, tree_type, container_type>* pParent;

	// friends
	#if defined(_MSC_VER) && _MSC_VER < 1300
		friend class basic_tree<stored_type, tree_type, container_type>;
		friend class associative_tree<stored_type, tree_type, container_type>;
		friend class const_pre_order_descendant_iterator<stored_type, tree_type, container_type, associative_tree<stored_type, tree_type, container_type> >;
		friend class const_post_order_descendant_iterator<stored_type, tree_type, container_type, associative_tree<stored_type, tree_type, container_type> >;
		friend bool operator == (const const_associative_iterator& lhs, const const_associative_iterator& rhs);
	#else
		template<typename T, typename U, typename V> friend class basic_tree;
		template<typename T, typename U, typename V> friend class associative_tree;
		template<typename T, typename U, typename V, typename W> friend class const_pre_order_descendant_iterator;
		template<typename T, typename U, typename V, typename W> friend class const_post_order_descendant_iterator;
		friend bool operator ==<> (const const_associative_iterator& lhs, const const_associative_iterator& rhs);
	#endif
};

template<typename stored_type, typename tree_type, typename container_type>
class tcl::associative_iterator : public tcl::const_associative_iterator<stored_type, tree_type, container_type>
{
private:
public:
	// constructors/destructor
	associative_iterator() : const_associative_iterator<stored_type, tree_type, container_type>() {}
private:
	explicit associative_iterator(typename container_type::iterator iter, associative_tree<stored_type, tree_type, container_type>* pCalled_node) : const_associative_iterator<stored_type, tree_type, container_type>(iter, pCalled_node) {}
	// destructor, copy constructor, and assignment operator will be compiler generated correctly

public:
	// overloaded operators
	stored_type& operator*() const { return *(*it)->get(); }
	stored_type* operator->() const { return (*it)->get(); }
	associative_iterator& operator ++() { ++it;  return *this; }
	associative_iterator operator ++(int) { associative_iterator old(*this); ++*this; return old; }
	associative_iterator& operator --() { --it; return *this; }
	associative_iterator operator --(int) { associative_iterator old(*this); --*this; return old; }

	tree_type* node() const { return *it; }

	// friends
	using const_associative_iterator<stored_type, tree_type, container_type>::it;
	#if defined(_MSC_VER) && _MSC_VER < 1300
		friend class associative_tree<stored_type, tree_type, container_type>;
		friend class associative_reverse_iterator<stored_type, tree_type, container_type>;
	#else
		template<typename T, typename U, typename V> friend class associative_tree;
		template<typename T, typename U, typename V> friend class associative_reverse_iterator;
	#endif
};

/************************************************************************/
/* sequential tree child iterators                                      */
/************************************************************************/


template<typename stored_type, typename tree_type, typename container_type>
class tcl::const_sequential_iterator : public std::iterator<std::random_access_iterator_tag, stored_type>
{
public:
	// constructors/destructor
	const_sequential_iterator() : pParent(0) {}
	virtual ~const_sequential_iterator() {}
protected:
	explicit const_sequential_iterator(typename container_type::const_iterator iter, const tree_type* pCalled_node) : it(iter), pParent(pCalled_node) {}
	// copy constructor, and assignment operator will be compiler generated correctly

public:
	// typedefs
	typedef size_t size_type;
	#if defined(_MSC_VER) && _MSC_VER < 1300
		typedef std::iterator_traits<std::iterator<std::random_access_iterator_tag, stored_type> >::distance_type difference_type;
	#else
		typedef typename std::iterator_traits<const_sequential_iterator>::difference_type difference_type;
	#endif

	// overloaded operators
	const stored_type& operator*() const { return  *(*it)->get(); }
	const stored_type* operator->() const { return (*it)->get(); }

	const_sequential_iterator& operator ++() { ++it; return *this; }
	const_sequential_iterator operator ++(int) { const_sequential_iterator old(*this); ++*this; return old; }
	const_sequential_iterator& operator --() { --it; return *this; }
	const_sequential_iterator operator --(int) { const_sequential_iterator old(*this); --*this; return old; }
	const_sequential_iterator& operator +=(difference_type n) { it += n; return *this; }
	const_sequential_iterator& operator -=(difference_type n) { it -= n; return *this; }
	difference_type operator -(const const_sequential_iterator& rhs) const { return it - rhs.it; }
	const tree_type* node() const { return *it; }

	// data
protected:
	typename container_type::const_iterator it;
	const tree_type* pParent;

	// friends 
	#if defined(_MSC_VER) && _MSC_VER < 1300
		friend class sequential_tree<stored_type>;
		friend class const_pre_order_descendant_iterator<stored_type, tree_type, container_type, sequential_tree<stored_type> >;
		friend class const_post_order_descendant_iterator<stored_type, tree_type, container_type, sequential_tree<stored_type> >;
		friend bool operator == (const const_sequential_iterator& lhs, const const_sequential_iterator& rhs );
		friend bool operator <  (const const_sequential_iterator& lhs, const const_sequential_iterator& rhs);
	#else
		friend bool operator ==<> (const const_sequential_iterator& lhs, const const_sequential_iterator& rhs );
		friend bool operator < <> (const const_sequential_iterator& lhs, const const_sequential_iterator& rhs);
		template<typename T> friend class sequential_tree;
		template<typename T, typename U, typename V, typename W> friend class const_pre_order_descendant_iterator;
		template<typename T, typename U, typename V, typename W> friend class const_post_order_descendant_iterator;
	#endif
};




template<typename stored_type, typename tree_type, typename container_type>
class tcl::sequential_iterator : public tcl::const_sequential_iterator<stored_type, tree_type, container_type>
{
public:
	// constructors/destructor
	sequential_iterator() : const_sequential_iterator<stored_type, tree_type, container_type>() {}
private:
	explicit sequential_iterator(const typename container_type::iterator& iter, const tree_type* const pCalled_node) : const_sequential_iterator<stored_type, tree_type, container_type>(iter, pCalled_node) {}
	// destructor, copy constructor, and assignment operator will be compiler generated correctly

public:
	// overloaded operators
	stored_type& operator*() const { return *(*it)->get(); }
	stored_type* operator->() const { return (*it)->get(); }
	sequential_iterator& operator ++() { ++it;  return *this; }
	sequential_iterator operator ++(int) { sequential_iterator old(*this); ++*this; return old; }
	sequential_iterator& operator --() { --it; return *this; }
	sequential_iterator operator --(int) { sequential_iterator old(*this); --*this; return old; }
	tree_type* node() const { return *it; }

	// friends
	using const_sequential_iterator<stored_type, tree_type, container_type>::it;
	#if defined(_MSC_VER) && _MSC_VER < 1300
		friend class sequential_tree<stored_type>;
		friend class sequential_reverse_iterator<stored_type, tree_type, container_type>;
	#else
		template<typename T> friend class sequential_tree;
		template<typename T, typename U, typename V> friend class sequential_reverse_iterator;
	#endif
};








