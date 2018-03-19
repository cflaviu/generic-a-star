#pragma once
/// A* Incremental Search Algorithm
/// Copyright (c) Flaviu Cibu. All rights reserved.
/// Created 06-sep-‎2009
/// Updated 26-nov-‎2010

namespace stdext {
namespace astar
{
	/// Base node class of the nodes used by the algorithm. This class ensure the existente of 3 values for each node:
	/// total score (f), general score (g) and heuristic score (h). 
	/// For more details see http://en.wikipedia.org/wiki/A*_search_algorithm#Algorithm_description.
	/// The descendant of this class has to provide two methods:
	/// the distance/cost to another node (signature Value distance_to( const note_type& node) const)
	/// and the estimated (heuristic) distance/cost to target node (signature void set_heuristic_score( const note_type& target_node)).
	template <typename _Score>
	class base_node
	{
	public:
		typedef _Score score_type;

		base_node():
			general_score_( 0),
			heuristic_score_( 0)
		{}

		base_node( const base_node& node):
			general_score_( node.general_score_),
			heuristic_score_( node.heuristic_score_)
		{}

		/// Gets the total score (f = g + h).
		score_type total_score() const { return general_score_ + heuristic_score_; }

		/// Gets the general score (g).
		score_type general_score() const { return general_score_; }

		/// Sets the general score (g).
		void set_general_score( const score_type value) { general_score_ = value; }

		/// Gets the heuristic score (h).
		score_type heuristic_score() const { return heuristic_score_; }

		/// Sets the heuristic score (h).
		void set_heuristic_score( const score_type value) { heuristic_score_ = value; }

		/// Operator < - used on sorting of the priority queue of open set.
		bool operator < ( const base_node& node) const { return total_score() < node.total_score(); }

	protected:
		score_type general_score_;
		score_type heuristic_score_;
	};


	/// Dummy beam search functor.
	struct no_beam_search
	{
		template <typename _Node, typename _PriorityOpenSet, typename _OpenSet>
		void operator () ( _Node&, _PriorityOpenSet&, _OpenSet&) {}
	};


	/// @brief Generic C++ implementation of A* algorithm (http://en.wikipedia.org/wiki/A*_search_algorithm).
	/// Features: Fully customizable internal data structures, step-by-step execution and beam search support.
	template <typename _Node, typename _PriorityQueue, typename _NeighborEnumerator, typename _Set, 
				typename _SolutionVerifier, typename _Solution, typename _BeamSearch = no_beam_search>
	class algo
	{
	public:
		typedef _Node				note_type;
		typedef _PriorityQueue		priority_queue_type;
		typedef _Set				set_type;
		typedef _NeighborEnumerator	neighbor_enumerator_type;
		typedef _SolutionVerifier	solution_verifier_type;
		typedef _Solution			solution_type;
		typedef _BeamSearch			beam_search_type;

		/// @param[in] start_node Start node
		/// @param[in] target_node Target node
		/// @param[in] solution_verifier solution_type verifier functor - checks if the current node is the target node.
		/// @param[in] neighbor_enumerator Enumerator of adjacent nodes
		/// @param[in] beam_search Beach search - filter of the items from open set. 
		/// See more details at http://theory.stanford.edu/~amitp/GameProgramming/Variations.html#S1.
		/// By default the beam search is disable using a dummy filter no_beam_search.
		algo( const note_type& start_node, const note_type& target_node, const solution_verifier_type& solution_verifier,
				const neighbor_enumerator_type& neighbor_enumerator = neighbor_enumerator_type(), const beam_search_type& beam_search = beam_search_type()):
			solution_verifier_( solution_verifier),
			beam_search_( beam_search),
			neighbor_enumerator_( neighbor_enumerator),
			target_node_( target_node),
			has_solution_( false)
		{
			priority_open_set.push( start_node);
			open_set_.insert( start_node);
		}

		/// Checks if the solution was found. If the return is true, the solution can be used by calling @ref solution method.
		bool has_solution() const { return has_solution_; }

		/// Gets the solution instance.
		const solution_type& solution() const { return solution_; }

		/// Gets the solution instance.
		solution_type& solution() { return solution_; }

		/// @brief algo progress method - useful for fined grained execution, early exit 
		/// (see http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html#S16)
		/// and algorithm interruption (see http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html#S17).
		/// @return Returns true if the algorith should continue. Otherwise the algorithm cannot continue and
		/// @ref has_solution method has to be checked.
		/// @remark If the method returns false and also @ref has_solution returns false then no solution was found.
		/// @note The implementation is based on pseudo code from http://en.wikipedia.org/wiki/A*_search_algorithm#Pseudo_code.
		bool operator() ()
		{
			bool can_continue = false;
			if ( !open_set_.empty())
			{
				node_ = priority_open_set.top();
				has_solution_ = solution_verifier_( node_);
				if ( !has_solution_)
				{
					can_continue = true;
					priority_open_set.pop();
					open_set_.erase( node_);
					closed_set_.insert( node_);

					note_type::score_type tentative_general_score;
					bool needTest;
					neighbor_enumerator_( node_);
					while( neighbor_enumerator_ && closed_set_.find( *neighbor_enumerator_) == closed_set_.end())
					{
						note_type& neighbor = *neighbor_enumerator_;
						tentative_general_score = node_.general_score() + node_.distance_to( neighbor);
						needTest = open_set_.find( neighbor) == open_set_.end();
						if ( needTest || tentative_general_score < neighbor.general_score())
						{
							neighbor.set_general_score( tentative_general_score);
							neighbor.set_heuristic_score( target_node_);
							if ( needTest)
							{
								priority_open_set.push( neighbor);
								open_set_.insert( neighbor);
								beam_search_( neighbor, priority_open_set, open_set_);
							}

							solution_.insert( typename solution_type::value_type( neighbor, node_));
						}
						++neighbor_enumerator_;
					}
				}
			}
			return can_continue;
		}

	protected:
		neighbor_enumerator_type	neighbor_enumerator_;
		solution_verifier_type		solution_verifier_;
		beam_search_type			beam_search_;
		priority_queue_type			priority_open_set;
		set_type					open_set_;
		set_type					closed_set_;
		solution_type				solution_;
		note_type					node_;
		note_type					target_node_;
		bool						has_solution_;
	};
}}
