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
#include "child_iterator.h"
#include <set>
#include <stack>
#include <queue>
#include <algorithm>

namespace tcl 
{
	template<typename T, typename U, typename V, typename W> class const_pre_order_descendant_iterator;
	template<typename T, typename U, typename V, typename W> class pre_order_descendant_iterator;
	template<typename T, typename U, typename V, typename W> class const_post_order_descendant_iterator;
	template<typename T, typename U, typename V, typename W> class post_order_descendant_iterator;
	template<typename T, typename U, typename V, typename W> class const_level_order_descendant_iterator;
	template<typename T, typename U, typename V, typename W> class level_order_descendant_iterator;
	template<typename T, typename U> class tree;
	template<typename T, typename U> class multitree;
	template<typename T, typename U, typename V> class unique_tree;

	// comparison operators
	template<typename T, typename U, typename V, typename W> bool operator == (const const_pre_order_descendant_iterator<T, U, V, W>& lhs, const const_pre_order_descendant_iterator<T, U, V, W>& rhs ) { return lhs.it == rhs.it && lhs.at_top == rhs.at_top; }
	template<typename T, typename U, typename V, typename W> bool operator != (const const_pre_order_descendant_iterator<T, U, V, W>& lhs, const const_pre_order_descendant_iterator<T, U, V, W>& rhs ) { return !(lhs == rhs); }
	template<typename T, typename U, typename V, typename W> bool operator == ( const const_post_order_descendant_iterator<T, U, V, W>& lhs, const const_post_order_descendant_iterator<T, U, V, W>& rhs ) { return lhs.it == rhs.it && lhs.at_top == rhs.at_top; }
	template<typename T, typename U, typename V, typename W> bool operator != ( const const_post_order_descendant_iterator<T, U, V, W>& lhs, const const_post_order_descendant_iterator<T, U, V, W>& rhs ) { return !(lhs == rhs); }
	template<typename T, typename U, typename V, typename W> bool operator == (const const_level_order_descendant_iterator<T, U, V, W>& lhs, const const_level_order_descendant_iterator<T, U, V, W>& rhs) { return lhs.it == rhs.it && lhs.at_top == rhs.at_top; }
	template<typename T, typename U, typename V, typename W> bool operator != (const const_level_order_descendant_iterator<T, U, V, W>& lhs, const const_level_order_descendant_iterator<T, U, V, W>& rhs) { return !(lhs == rhs); }
}


/************************************************************************/
/* descendant iterators                                                 */
/************************************************************************/

template<typename stored_type, typename tree_type, typename container_type, typename tree_category_type>
class tcl::const_pre_order_descendant_iterator : public std::iterator<std::bidirectional_iterator_tag, stored_type>
{
public:
	// constructors/destructor
	const_pre_order_descendant_iterator() : pTop_node(0), at_top(false) {}
	virtual ~const_pre_order_descendant_iterator() {}
	//  copy constructor, and assignment operator will be compiler generated correctly
protected:
	explicit const_pre_order_descendant_iterator(const tree_type* pCalled_node, bool beg) : it(beg ? pCalled_node->begin() : pCalled_node->end()), pTop_node(pCalled_node), at_top(beg) {}

public:
	// overloaded operators
	const_pre_order_descendant_iterator& operator ++(); 
	const_pre_order_descendant_iterator operator ++(int) { const_pre_order_descendant_iterator old(*this); ++*this; return old; }
	const_pre_order_descendant_iterator& operator --();
	const_pre_order_descendant_iterator operator --(int) { const_pre_order_descendant_iterator old(*this); --*this; return old; }

	// public interface
	const stored_type& operator*() const { return  at_top ? *pTop_node->get() : it.operator *(); }
	const stored_type* operator->() const { return at_top ? pTop_node->get() : it.operator ->(); }
	const tree_type* node() const { return at_top ? pTop_node : it.node(); }

	// data
protected:
	typename tree_category_type::const_iterator it;
	std::stack<typename tree_category_type::const_iterator> node_stack;   
	const tree_type* pTop_node;
	typename container_type::const_reverse_iterator rit;
	bool at_top;

	// friends
	#if defined(_MSC_VER) && _MSC_VER < 1300
    typedef tree_type tr_type;
		friend typename tr_type;
		friend class sequential_tree<stored_type>;
		friend bool operator == (const const_pre_order_descendant_iterator& lhs, const const_pre_order_descendant_iterator& rhs );
	#else
		template<typename T, typename U> friend class tree;
		template<typename T, typename U> friend class multitree;
		template<typename T, typename U, typename V> friend class unique_tree;
		template<typename T> friend class sequential_tree;
		friend bool operator ==<> (const const_pre_order_descendant_iterator& lhs, const const_pre_order_descendant_iterator& rhs );
	#endif
};

template<typename stored_type, typename tree_type, typename container_type, typename tree_category_type>
class tcl::pre_order_descendant_iterator : public tcl::const_pre_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>
{
public:
	// constructors/destructor
	pre_order_descendant_iterator() : const_pre_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>() {}
	// destructor, copy constructor, and assignment operator will be compiler generated correctly
protected:
	explicit pre_order_descendant_iterator(const tree_type* const pCalled_node, bool beg) : const_pre_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>(pCalled_node, beg) {}

public:
	// overloaded operators
	pre_order_descendant_iterator& operator ++() { ++(*static_cast<const_pre_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>*>(this)); return *this; }
	pre_order_descendant_iterator operator ++(int) { pre_order_descendant_iterator old(*this); ++*this; return old; }
	pre_order_descendant_iterator& operator --() { --(*static_cast<const_pre_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>*>(this)); return *this; }
	pre_order_descendant_iterator operator --(int) { pre_order_descendant_iterator old(*this); --*this; return old; }

	// public interface
	stored_type& operator*() const { return at_top ? const_cast<stored_type&>(*pTop_node->get()) : const_cast<stored_type&>(it.operator *()); }
	stored_type* operator->() const { return at_top ? const_cast<stored_type*>(pTop_node->get()) : const_cast<stored_type*>(it.operator ->()); }
	tree_type* node() const { return at_top ? const_cast<tree_type*>(pTop_node) : const_cast<tree_type*>(it.node()); }

	// friends
	using const_pre_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>::it;
	using const_pre_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>::pTop_node;
	using const_pre_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>::at_top;
	#if defined(_MSC_VER) && _MSC_VER < 1300
    typedef tree_type tr_type;
		friend typename tr_type;
		friend class sequential_tree<stored_type>;
	#else
		template<typename T, typename U> friend class tree;
		template<typename T, typename U> friend class multitree;
		template<typename T, typename U, typename V> friend class unique_tree;
		template<typename T> friend class sequential_tree;
	#endif
};

template<typename stored_type, typename tree_type, typename container_type, typename tree_category_type>
class tcl::const_post_order_descendant_iterator : public std::iterator<std::bidirectional_iterator_tag, stored_type>
{
public:
	// constructors/destructor
	const_post_order_descendant_iterator() : pTop_node(0) {}
	virtual ~const_post_order_descendant_iterator() {}
	// copy constructor, and assignment operator will be compiler generated correctly
protected:
	explicit const_post_order_descendant_iterator(const tree_type* pCalled_node, bool beg); 

public:
	// overloaded operators
	const_post_order_descendant_iterator& operator ++(); 
	const_post_order_descendant_iterator operator ++(int) { const_post_order_descendant_iterator old(*this); ++*this; return old; }
	const_post_order_descendant_iterator& operator --(); 
	const_post_order_descendant_iterator operator --(int) { const_post_order_descendant_iterator old(*this); --*this; return old; }

	// public interface
	const stored_type& operator*() const { return at_top ? *pTop_node->get() : it.operator *(); }
	const stored_type* operator->() const { return at_top ? pTop_node->get() : it.operator ->(); }
	const tree_type* node() const { return at_top ? pTop_node : it.node(); }

	// data
protected:
	std::stack<typename tree_category_type::const_iterator > node_stack;   
	typename tree_category_type::const_iterator it;
	const tree_type* pTop_node;
	typename container_type::const_reverse_iterator rit;
	bool at_top;

	// friends
	#if defined(_MSC_VER) && _MSC_VER < 1300
		friend class sequential_tree<stored_type>;
    typedef tree_type tr_type;
		friend typename tr_type;
		friend bool operator == ( const const_post_order_descendant_iterator& lhs, const const_post_order_descendant_iterator& rhs );
	#else
		template<typename T> friend class sequential_tree;
		template<typename T, typename U> friend class tree;
		template<typename T, typename U> friend class multitree;
		template<typename T, typename U, typename V> friend class unique_tree;
		friend bool operator ==<> ( const const_post_order_descendant_iterator& lhs, const const_post_order_descendant_iterator& rhs );
	#endif
};

template<typename stored_type, typename tree_type, typename container_type, typename tree_category_type>
class tcl::post_order_descendant_iterator : public tcl::const_post_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>
{
public:
	// constructors/destructor
	post_order_descendant_iterator() : const_post_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>() {}
	// destructor, copy constructor, and assignment operator will be compiler generated correctly
protected:
	explicit post_order_descendant_iterator(const tree_type* pCalled_node, bool beg) : const_post_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>(pCalled_node, beg) { }

public:
	// overloaded operators
	post_order_descendant_iterator& operator ++() { ++(*static_cast<const_post_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>*>(this)); return *this; }
	post_order_descendant_iterator operator ++(int) { post_order_descendant_iterator old(*this); ++*this; return old; }
	post_order_descendant_iterator& operator --() { --(*static_cast<const_post_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>*>(this)); return *this; }
	post_order_descendant_iterator operator --(int) { post_order_descendant_iterator old(*this); --*this; return old; }

	// public interface
	stored_type& operator*() const { return at_top ? const_cast<stored_type&>(*pTop_node->get()) : const_cast<stored_type&>(it.operator *()); }
	stored_type* operator->() const { return at_top ? const_cast<stored_type*>(pTop_node->get()) : const_cast<stored_type*>(it.operator ->()); }
	tree_type* node() const { return at_top ? const_cast<tree_type*>(pTop_node) : const_cast<tree_type*>(it.node()); }

	// friends
	using const_post_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>::it;
	using const_post_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>::pTop_node;
	using const_post_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>::at_top;
	#if defined(_MSC_VER) && _MSC_VER < 1300
    typedef tree_type tr_type;
		friend typename tr_type;
		friend class sequential_tree<stored_type>;
	#else
		template<typename T, typename U> friend class tree;
		template<typename T, typename U> friend class multitree;
		template<typename T, typename U, typename V> friend class unique_tree;
		template<typename T> friend class sequential_tree;
	#endif
};

template<typename stored_type, typename tree_type, typename container_type, typename tree_category_type>
class tcl::const_level_order_descendant_iterator : public std::iterator<std::forward_iterator_tag, stored_type>
{
public:
	// constructors/destructor
	const_level_order_descendant_iterator() : pTop_node(0), at_top(false) {}
	virtual ~const_level_order_descendant_iterator() {}
	// copy constructor, and assignment operator will be compiler generated correctly
protected:
	explicit const_level_order_descendant_iterator(const tree_type* pCalled_node, bool beg) : it(beg ? pCalled_node->begin() : pCalled_node->end()), pTop_node(pCalled_node), at_top(beg) {}

public:
	// overloaded operators
	const_level_order_descendant_iterator& operator ++();
	const_level_order_descendant_iterator operator ++(int) { const_level_order_descendant_iterator old(*this); ++*this; return old; }

	// public interface
	const stored_type& operator*() const { return at_top ? *pTop_node->get() : it.operator *(); }
	const stored_type* operator->() const { return at_top ? pTop_node->get() : it.operator ->(); }
	const tree_type* node() const { return at_top ? pTop_node : it.node(); }

	// data
protected:
	typename tree_category_type::const_iterator it;
	std::queue<typename tree_category_type::const_iterator> node_queue;
	const tree_type* pTop_node;
	bool at_top;

	// friends
	#if defined(_MSC_VER) && _MSC_VER < 1300
    typedef tree_type tr_type;
		friend typename tr_type;
		friend class sequential_tree<stored_type>;
		friend bool operator == (const const_level_order_descendant_iterator& lhs, const const_level_order_descendant_iterator& rhs);
	#else
		template<typename T, typename U> friend class tree;
		template<typename T, typename U> friend class multitree;
		template<typename T, typename U, typename V> friend class unique_tree;
		template<typename T> friend class sequential_tree;
		friend bool operator ==<> (const const_level_order_descendant_iterator& lhs, const const_level_order_descendant_iterator& rhs);
	#endif
};

template<typename stored_type, typename tree_type, typename container_type, typename tree_category_type>
class tcl::level_order_descendant_iterator : public tcl::const_level_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>
{
public:
	// constructors/destructor
	level_order_descendant_iterator() : const_level_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>() {}
	// destructor, copy constructor, and assignment operator will be compiler generated correctly
protected:
	explicit level_order_descendant_iterator(const tree_type* const pCalled_node, bool beg) : const_level_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>(pCalled_node, beg) {}

public:
	// overloaded operators
	level_order_descendant_iterator& operator ++() { ++(*static_cast<const_level_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>*>(this)); return *this; }
	level_order_descendant_iterator operator ++(int) { level_order_descendant_iterator old(*this); ++*this; return old; }

	// public interface
	stored_type& operator*() const { return at_top ? const_cast<stored_type&>(*pTop_node->get()) : const_cast<stored_type&>(it.operator *()); }
	stored_type* operator->() const { return at_top ? const_cast<stored_type*>(pTop_node->get()) : const_cast<stored_type*>(it.operator ->()); }
	tree_type* node() const { return at_top ? const_cast<tree_type*>(pTop_node) : const_cast<tree_type*>(it.node()); }

	// friends
	using const_level_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>::it;
	using const_level_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>::pTop_node;
	using const_level_order_descendant_iterator<stored_type, tree_type, container_type, tree_category_type>::at_top;
	#if defined(_MSC_VER) && _MSC_VER < 1300
    typedef tree_type tr_type;
		friend typename tr_type;
		friend class sequential_tree<stored_type>;
	#else
		template<typename T, typename U> friend class multitree;
		template<typename T, typename U, typename V> friend class unique_tree;
		template<typename T, typename U> friend class tree;
		template<typename T> friend class sequential_tree;
	#endif
};


#include "descendant_iterator.inl"
