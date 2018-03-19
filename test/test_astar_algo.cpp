//#include "stdafx.h"
#include "astar_algo.h"
#include <vector>
#include <queue>
#include <set>
#include <iostream>

using namespace std;
using namespace stdext;

class XyNode: public astar::base_node<int>
{
	public:
		typedef int Value;
		typedef astar::base_node<int> Base;

		XyNode( const int id = 0, const Value x = 0, const Value y = 0):
			iID( id),
			iX( x),
			iY( y)
		{}

		XyNode( const XyNode& node):
			Base( node),
			iID( node.iID),
			iX( node.iX),
			iY( node.iY),
			neighbors( node.neighbors)
		{}

		operator int() const { return id(); }

		int id() const { return iID; }
		void setID( const int value) { iID = value; }

		Value x() const { return iX; }
		void setX( const Value value) { iX = value; }

		Value y() const { return iY; }
		void setY( const Value value) { iY = value; }

		Value distance_to( const XyNode& node) const
		{
			return iX * node.iX + iY + node.iY;
		}

		void set_heuristic_score( const XyNode& targetNode) { Base::set_heuristic_score( distance_to( targetNode)); }

		typedef std::vector<int> Neighbors;

		Neighbors neighbors;

	protected:
		Value iID, iX, iY;
};


struct SolutionVerifier
{
	const int iNodeID;
	SolutionVerifier( const int nodeID): iNodeID( nodeID) {}

	bool operator() ( const XyNode& n) const
	{
		return n.id() == iNodeID;
	}
};


typedef vector<XyNode> NodeList;

class Enumerator
{
public:
	Enumerator( NodeList& nodeList): 
		node_( 0), 
		iNodeList( nodeList) 
	{}

	operator bool () const { return node_ != 0; }

	void operator () ( const XyNode& node)
	{
		iNeighbors = node.neighbors.begin();
		iNeighborsEnd = node.neighbors.end();
		if ( iNeighbors == iNeighborsEnd)
		{
			node_ = 0;
		}
		else
		{
			node_ = &iNodeList[ *iNeighbors];
		}
	}

	void operator ++ ()
	{
		if ( node_)
		{
			if ( iNeighbors + 1 != iNeighborsEnd)
			{
				++iNeighbors;
				node_ = &iNodeList[ *iNeighbors];
			}
			else
			{
				node_ = 0;
			}
		}
	}

	XyNode& operator * () { return *node_; }
	const XyNode& operator * () const { return *node_; }

	XyNode* operator -> () { return node_; }
	const XyNode* operator -> () const { return node_; }

private:
	XyNode::Neighbors::const_iterator	iNeighbors;
	XyNode::Neighbors::const_iterator	iNeighborsEnd;
	XyNode*							node_;
	NodeList&						iNodeList;
};


void test1( NodeList& list)
{
	list.clear();

	XyNode n;

	n.setID( 0);
	n.setX( 0);
	n.setY( 5);
	n.neighbors.clear();
	n.neighbors.push_back( 1);
	n.neighbors.push_back( 2);
	list.push_back( n);

	n.setID( 1);
	n.setX( 3);
	n.setY( 6);
	n.neighbors.clear();
	n.neighbors.push_back( 3);
	list.push_back( n);

	n.setID( 2);
	n.setX( 4);
	n.setY( 3);
	n.neighbors.clear();
	n.neighbors.push_back( 4);
	n.neighbors.push_back( 5);
	list.push_back( n);

	n.setID( 3);
	n.setX( 6);
	n.setY( 9);
	n.neighbors.clear();
	n.neighbors.push_back( 6);
	n.neighbors.push_back( 7);
	list.push_back( n);

	n.setID( 4);
	n.setX( 7);
	n.setY( 3);
	n.neighbors.clear();
	n.neighbors.push_back( 8);
	n.neighbors.push_back( 10);
	list.push_back( n);

	n.setID( 5);
	n.setX( 6);
	n.setY( 1);
	n.neighbors.clear();
	n.neighbors.push_back( 8);
	list.push_back( n);

	n.setID( 6);
	n.setX( 8);
	n.setY( 6);
	n.neighbors.clear();
	n.neighbors.push_back( 7);
	n.neighbors.push_back( 10);
	list.push_back( n);

	n.setID( 7);
	n.setX( 11);
	n.setY( 8);
	n.neighbors.clear();
	n.neighbors.push_back( 9);
	list.push_back( n);

	n.setID( 8);
	n.setX( 10);
	n.setY( 2);
	n.neighbors.clear();
	n.neighbors.push_back( 11);
	list.push_back( n);

	n.setID( 9);
	n.setX( 13);
	n.setY( 6);
	n.neighbors.clear();
	list.push_back( n);

	n.setID( 10);
	n.setX( 8);
	n.setY( 6);
	n.neighbors.clear();
	n.neighbors.push_back( 12);
	list.push_back( n);

	n.setID( 11);
	n.setX( 13);
	n.setY( 0);
	n.neighbors.clear();
	list.push_back( n);

	n.setID( 12);
	n.setX( 17);
	n.setY( 3);
	n.neighbors.clear();
	list.push_back( n);
}


struct SolutionItem
{
	const int from;
	const int to;

	SolutionItem( const int from, const int to = 0): from( from), to( to) {}

	bool operator < ( const SolutionItem& item) const { return from < item.from; }
	bool operator == ( const SolutionItem& item) const { return from == item.from; }
};


typedef set<SolutionItem> Solution;

typedef astar::algo<XyNode, priority_queue<XyNode>, Enumerator, set<int>, SolutionVerifier, Solution> AStarAlgo;

void print( const Solution& s, const XyNode& targetNode)
{
	int id = targetNode;
	while( 1)
	{
		Solution::const_iterator item = s.find( id);
		if ( item != s.end())
		{
			cout << id << ' ';
			id = item->to;
		}
		else
			break;
	}
	cout << '\n';
}


int main()
{
	NodeList nodeList;
	test1( nodeList);

	const XyNode& startNode = nodeList[ 0];
	const XyNode& targetNode = nodeList[ nodeList.size() - 1];

	AStarAlgo as( startNode, targetNode, SolutionVerifier( targetNode), Enumerator( nodeList));

	int steps = 0;
	while( as())
	{ ++steps; }
	
	if ( as.has_solution())
	{
		cout << "steps=" << steps << " path: ";
		print( as.solution(), targetNode);
	}

	return 0;
}
