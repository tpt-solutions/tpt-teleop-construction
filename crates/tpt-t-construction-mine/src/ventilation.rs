// SPDX-License-Identifier: MIT OR Apache-2.0
// Copyright (c) 2026 TPT Solutions

//! Ventilation-aware routing (spec.txt §6): the mine's drivable tunnels
//! form a graph, and a route through it must avoid any segment whose
//! measured gas concentration exceeds the safe threshold — Dijkstra's
//! shortest path, but treating an unsafe edge as simply not present in
//! the graph rather than penalizing it, since no distance saved is worth
//! trading into an unsafe atmosphere.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// One drivable tunnel segment from its owning node.
#[derive(Debug, Clone, Copy)]
pub struct Edge {
    pub to: usize,
    pub distance_m: f32,
    pub gas_concentration_ppm: f32,
}

/// The mine's tunnel network as an adjacency list.
#[derive(Debug, Clone, Default)]
pub struct VentilationGraph {
    pub adjacency: Vec<Vec<Edge>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct HeapEntry {
    cost_m: f32,
    node: usize,
}

impl Eq for HeapEntry {}

impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reversed so `BinaryHeap` (a max-heap) pops the lowest cost first.
        other
            .cost_m
            .partial_cmp(&self.cost_m)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Finds the shortest route from `start` to `goal` using only edges whose
/// gas concentration is at or below `max_gas_ppm`, via Dijkstra's
/// algorithm. Returns the node sequence and total distance, or `None` if
/// no safe route exists.
pub fn shortest_safe_route(
    graph: &VentilationGraph,
    start: usize,
    goal: usize,
    max_gas_ppm: f32,
) -> Option<(Vec<usize>, f32)> {
    let n = graph.adjacency.len();
    let mut best_cost = vec![f32::INFINITY; n];
    let mut previous = vec![None; n];
    let mut heap = BinaryHeap::new();

    best_cost[start] = 0.0;
    heap.push(HeapEntry {
        cost_m: 0.0,
        node: start,
    });

    while let Some(HeapEntry { cost_m, node }) = heap.pop() {
        if node == goal {
            break;
        }
        if cost_m > best_cost[node] {
            continue;
        }
        for edge in &graph.adjacency[node] {
            if edge.gas_concentration_ppm > max_gas_ppm {
                continue;
            }
            let next_cost = cost_m + edge.distance_m;
            if next_cost < best_cost[edge.to] {
                best_cost[edge.to] = next_cost;
                previous[edge.to] = Some(node);
                heap.push(HeapEntry {
                    cost_m: next_cost,
                    node: edge.to,
                });
            }
        }
    }

    if best_cost[goal].is_infinite() {
        return None;
    }

    let mut path = vec![goal];
    let mut current = goal;
    while let Some(prev) = previous[current] {
        path.push(prev);
        current = prev;
    }
    path.reverse();
    Some((path, best_cost[goal]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn takes_the_direct_route_when_everything_is_safe() {
        let graph = VentilationGraph {
            adjacency: vec![
                vec![Edge {
                    to: 1,
                    distance_m: 10.0,
                    gas_concentration_ppm: 5.0,
                }],
                vec![],
            ],
        };
        let (path, cost) = shortest_safe_route(&graph, 0, 1, 50.0).unwrap();
        assert_eq!(path, vec![0, 1]);
        assert_eq!(cost, 10.0);
    }

    #[test]
    fn detours_around_a_high_gas_segment() {
        // 0 -> 1 direct (short, unsafe); 0 -> 2 -> 1 (longer, safe).
        let graph = VentilationGraph {
            adjacency: vec![
                vec![
                    Edge {
                        to: 1,
                        distance_m: 5.0,
                        gas_concentration_ppm: 200.0,
                    },
                    Edge {
                        to: 2,
                        distance_m: 8.0,
                        gas_concentration_ppm: 5.0,
                    },
                ],
                vec![],
                vec![Edge {
                    to: 1,
                    distance_m: 8.0,
                    gas_concentration_ppm: 5.0,
                }],
            ],
        };
        let (path, cost) = shortest_safe_route(&graph, 0, 1, 50.0).unwrap();
        assert_eq!(path, vec![0, 2, 1]);
        assert!((cost - 16.0).abs() < 1e-4);
    }

    #[test]
    fn no_safe_route_returns_none() {
        let graph = VentilationGraph {
            adjacency: vec![
                vec![Edge {
                    to: 1,
                    distance_m: 5.0,
                    gas_concentration_ppm: 200.0,
                }],
                vec![],
            ],
        };
        assert_eq!(shortest_safe_route(&graph, 0, 1, 50.0), None);
    }

    #[test]
    fn unreachable_goal_returns_none() {
        let graph = VentilationGraph {
            adjacency: vec![vec![], vec![]],
        };
        assert_eq!(shortest_safe_route(&graph, 0, 1, 50.0), None);
    }

    #[test]
    fn start_equals_goal_is_a_zero_length_route() {
        let graph = VentilationGraph {
            adjacency: vec![vec![]],
        };
        let (path, cost) = shortest_safe_route(&graph, 0, 0, 50.0).unwrap();
        assert_eq!(path, vec![0]);
        assert_eq!(cost, 0.0);
    }
}
