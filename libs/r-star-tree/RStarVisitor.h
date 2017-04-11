/*
 *  Copyright (c) 2008 Dustin Spicuzza <dustin@virtualroadside.com>
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of version 2.1 of the GNU Lesser General Public
 *  License as published by the Free Software Foundation.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 * 
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
 
 #ifndef RSTARVISITOR_H
 #define RSTARVISITOR_H
 
 #include "RStarBoundingBox.h"
 
 /**
	\file
	
	I'm not convinced that these are really the best way to implement
	this, but it works so I'll stick with it for the moment
	
	It should be noted that all of these items are typedef'ed inside 
	of the RStarTree class, so you shouldn't generally need to
	directly use them. 
 */
 

/********************************************************************
 * These are all 'acceptor' functors used for queries and removals, 
 * which will have the following characteristics:
 *
 * template<typename Node, typename Leaf>
 *
 *	bool operator()(const Node * node)
 *		-- returns true if this branch should be visited
 *
 *	bool operator()(const Leaf * leaf)
 *		-- returns true if this leaf should be visited
 *
 * This class of functions should be easy to copy, and are expected 
 * to be const. They are only used to determine whether something 
 * should be visited, and not do the actual visiting.
 * 
 ********************************************************************/

// returns true if the node overlaps the specified bound
template <typename Node, typename Leaf>
struct RStarAcceptOverlapping
{
	const typename Node::BoundingBox &m_bound;
	explicit RStarAcceptOverlapping(const typename Node::BoundingBox &bound) : m_bound(bound) {}
	
	bool operator()(const Node * const node) const 
	{ 
		return m_bound.overlaps(node->bound);
	}
	
	bool operator()(const Leaf * const leaf) const 
	{ 
		return m_bound.overlaps(leaf->bound); 
	}
	
	private: RStarAcceptOverlapping(){}
};


// returns true if the compared boundary is within the specified bound
template <typename Node, typename Leaf>
struct RStarAcceptEnclosing
{
	const typename Node::BoundingBox &m_bound;
	explicit RStarAcceptEnclosing(const typename Node::BoundingBox &bound) : m_bound(bound) {}
	
	bool operator()(const Node * const node) const 
	{ 
		return m_bound.overlaps(node->bound);
	}
	
	bool operator()(const Leaf * const leaf) const 
	{ 
		return m_bound.encloses(leaf->bound); 
	}
	
	private: RStarAcceptEnclosing(){}
};


// will always return true, no matter what
template <typename Node, typename Leaf>
struct RStarAcceptAny
{
	bool operator()(const Node * const node) const { return true; }
	bool operator()(const Leaf * const leaf) const { return true; }
};
 
 
/********************************************************************
 * These are all 'visitor' styled functions -- even though these are
 * specifically targeted for removal tasks, visitor classes are 
 * specified exactly the same way. 
 *
 * bool operator()(RStarLeaf<LeafType, dimensions> * leaf)
 * 		-- Removal: if returns true, then remove the node
 *		-- Visitor: return can actually be void, not used
 *
 * bool ContinueVisiting; (not a function)
 *		-- if false, then the query will end as soon as possible. It
 *		is not guaranteed that the operator() will not be called, so
 *		items may be removed/visited after this is set to false
 *
 * You may modify the items that the leaf points to, but under no
 * circumstance should the bounds of the item be modified (since
 * that would screw up the tree). 
 * 
 ********************************************************************/
 
 
/*
	Default functor used to delete nodes from the R* tree. You can specify 
	a different functor to use, as long as it has the same signature as this. 
*/
template <typename Leaf>
struct RStarRemoveLeaf{

	const bool ContinueVisiting;
	RStarRemoveLeaf() : ContinueVisiting(true) {}

	bool operator()(const Leaf * const leaf) const
	{
		return true; 
	}
};


// returns true if the specific leaf is matched. If remove duplicates is true, 
// then it searches for all possible instances of the item
template <typename Leaf>
struct RStarRemoveSpecificLeaf
{
	mutable bool ContinueVisiting;
	bool m_remove_duplicates;
	const typename Leaf::leaf_type &m_leaf;
	
	explicit RStarRemoveSpecificLeaf(const typename Leaf::leaf_type &leaf, bool remove_duplicates = false) : 
		ContinueVisiting(true), m_remove_duplicates(remove_duplicates), m_leaf(leaf) {}
		
	bool operator()(const Leaf * const leaf) const
	{
		if (ContinueVisiting && m_leaf == leaf->leaf)
		{
			if (!m_remove_duplicates)
				ContinueVisiting = false;
			return true;
		}
		return false;
	}
	
	private: RStarRemoveSpecificLeaf(){}
};


#endif
