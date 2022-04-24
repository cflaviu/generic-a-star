/// A* Incremental Search Algorithm
/// Copyright (c) Flaviu Cibu. All rights reserved.
/// Created 06-sep-‎2009
/// Updated 26-nov-‎2010
/// Updated 24-apr-‎2022
#pragma once
#ifndef PCH
    #include <utility>
#endif

namespace stdext::astar
{
    /// Base node class of the nodes used by the algorithm. This class ensure the
    /// existente of 3 values for each node: total score (f), general score (g) and
    /// heuristic score (h). For more details see
    /// http://en.wikipedia.org/wiki/A*_search_algorithm#Algorithm_description. The
    /// descendant of this class has to provide two methods: the distance/cost to
    /// another node (signature Value distance_to( const note_type& node) const) and
    /// the estimated (heuristic) distance/cost to target node (signature void
    /// set_heuristic_score( const note_type& target_node)).
    template <typename _Score>
    class base_node
    {
    public:
        using score_type = _Score;

        base_node() = default;
        base_node(const base_node& node) = default;

        /// Gets the total score (f = g + h).
        score_type total_score() const noexcept { return general_score_ + heuristic_score_; }

        /// Gets the general score (g).
        score_type general_score() const noexcept { return general_score_; }

        /// Sets the general score (g).
        void set_general_score(const score_type value) noexcept { general_score_ = value; }

        /// Gets the heuristic score (h).
        score_type heuristic_score() const noexcept { return heuristic_score_; }

        /// Sets the heuristic score (h).
        void set_heuristic_score(const score_type value) noexcept { heuristic_score_ = value; }

        /// Operator < - used on sorting of the priority queue of open set.
        bool operator<(const base_node& node) const noexcept { return total_score() < node.total_score(); }

    protected:
        score_type general_score_ {};
        score_type heuristic_score_ {};
    };

    /// Dummy beam search functor.
    struct no_beam_search
    {
        void operator()(auto&, auto&, auto&) const noexcept {}
    };

    /// @brief Generic C++ implementation of A* algorithm
    /// (http://en.wikipedia.org/wiki/A*_search_algorithm). Features: Fully
    /// customizable internal data structures, step-by-step execution and beam
    /// search support.
    template <typename _Node, typename _PriorityQueue, typename _NeighborEnumerator, typename _Set, typename _SolutionVerifier,
              typename _Solution, typename _BeamSearch = no_beam_search>
    class algo
    {
    public:
        using note_type = _Node;
        using priority_queue_type = _PriorityQueue;
        using set_type = _Set;
        using neighbor_enumerator_type = _NeighborEnumerator;
        using solution_verifier_type = _SolutionVerifier;
        using solution_type = _Solution;
        using beam_search_type = _BeamSearch;

        /// @param[in] start_node Start node
        /// @param[in] target_node Target node
        /// @param[in] solution_verifier solution_type verifier functor - checks if
        /// the current node is the target node.
        /// @param[in] neighbor_enumerator Enumerator of adjacent nodes
        /// @param[in] beam_search Beach search - filter of the items from open set.
        /// See more details at
        /// http://theory.stanford.edu/~amitp/GameProgramming/Variations.html#S1. By
        /// default the beam search is disable using a dummy filter no_beam_search.
        algo(note_type start_node, note_type target_node, solution_verifier_type solution_verifier,
             neighbor_enumerator_type neighbor_enumerator, beam_search_type beam_search):
            _solution_verifier(std::move(solution_verifier)),
            _beam_search(std::move(beam_search)),
            _neighbor_enumerator(std::move(neighbor_enumerator)),
            _target_node(std::move(target_node))
        {
            _priority_open_set.emplace(std::move(start_node));
            _open_set.emplace(std::move(start_node));
        }

        /// Checks if the solution was found. If the return is true, the solution can
        /// be used by calling @ref solution method.
        bool has_solution() const noexcept { return _has_solution; }

        /// Gets the solution instance.
        const solution_type& solution() const noexcept { return _solution; }

        /// Gets the solution instance.
        solution_type& solution() noexcept { return _solution; }

        /// @brief algo progress method - useful for fined grained execution, early
        /// exit (see
        /// http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html#S16)
        /// and algorithm interruption (see
        /// http://theory.stanford.edu/~amitp/GameProgramming/ImplementationNotes.html#S17).
        /// @return Returns true if the algorith should continue. Otherwise the
        /// algorithm cannot continue and
        /// @ref has_solution method has to be checked.
        /// @remark If the method returns false and also @ref has_solution returns
        /// false then no solution was found.
        /// @note The implementation is based on pseudo code from
        /// http://en.wikipedia.org/wiki/A*_search_algorithm#Pseudo_code.
        bool operator()()
        {
            bool can_continue = false;
            if (!_open_set.empty())
            {
                _node = _priority_open_set.top();
                _has_solution = _solution_verifier(_node);
                if (!_has_solution)
                {
                    can_continue = true;
                    _priority_open_set.pop();
                    _open_set.erase(_node);
                    _closed_set.insert(_node);
                    _neighbor_enumerator(_node);
                    while (_neighbor_enumerator && _closed_set.find(*_neighbor_enumerator) == _closed_set.end())
                    {
                        note_type& neighbor = *_neighbor_enumerator;
                        auto tentative_general_score = _node.general_score() + _node.distance_to(neighbor);
                        const bool need_test = _open_set.find(neighbor) == _open_set.end();
                        if (need_test || (tentative_general_score < neighbor.general_score()))
                        {
                            neighbor.set_general_score(tentative_general_score);
                            neighbor.set_heuristic_score(_target_node);
                            if (need_test)
                            {
                                _priority_open_set.push(neighbor);
                                _open_set.insert(neighbor);
                                _beam_search(neighbor, _priority_open_set, _open_set);
                            }

                            _solution.insert({neighbor, _node});
                        }

                        ++_neighbor_enumerator;
                    }
                }
            }

            return can_continue;
        }

    protected:
        solution_verifier_type _solution_verifier;
        beam_search_type _beam_search;
        neighbor_enumerator_type _neighbor_enumerator;
        priority_queue_type _priority_open_set;
        set_type _open_set;
        set_type _closed_set;
        solution_type _solution;
        note_type _node;
        note_type _target_node;
        bool _has_solution {};
    };
} // namespace stdext::astar
