//! Minimal Polyanya harness: read a map and query pairs (JSON), build the navigation mesh,
//! answer the queries, write timings and path lengths (JSON) to stdout.
//!
//! Input:  {"boundary": [[x, y], ...], "holes": [[[x, y], ...], ...], "queries": [[sx, sy, gx, gy], ...]}
//! Output: {"triangulate_s", "merge_s", "bake_s", "polygons", "query_s": [...], "lengths": [... | null]}
use glam::Vec2;
use polyanya::Triangulation;
use serde::{Deserialize, Serialize};
use std::time::Instant;

#[derive(Deserialize)]
struct Input {
    boundary: Vec<[f64; 2]>,
    holes: Vec<Vec<[f64; 2]>>,
    queries: Vec<[f64; 4]>,
}

#[derive(Serialize)]
struct Output {
    triangulate_s: f64,
    merge_s: f64,
    bake_s: f64,
    polygons: usize,
    query_s: Vec<f64>,
    lengths: Vec<Option<f64>>,
}

fn v(p: &[f64; 2]) -> Vec2 {
    Vec2::new(p[0] as f32, p[1] as f32)
}

fn main() {
    let input: Input = serde_json::from_reader(std::io::stdin().lock()).expect("invalid input");

    let t = Instant::now();
    let outer: Vec<Vec2> = input.boundary.iter().map(v).collect();
    let mut triangulation = Triangulation::from_outer_edges(&outer);
    triangulation.add_obstacles(input.holes.iter().map(|h| h.iter().map(v).collect::<Vec<_>>()));
    let mut mesh = triangulation.as_navmesh();
    let triangulate_s = t.elapsed().as_secs_f64();

    let t = Instant::now();
    while mesh.merge_polygons() {}
    let merge_s = t.elapsed().as_secs_f64();

    let t = Instant::now();
    mesh.bake();
    let bake_s = t.elapsed().as_secs_f64();

    let mut query_s = Vec::with_capacity(input.queries.len());
    let mut lengths = Vec::with_capacity(input.queries.len());
    for q in &input.queries {
        let (start, goal) = (v(&[q[0], q[1]]), v(&[q[2], q[3]]));
        let t = Instant::now();
        let path = mesh.path(start, goal);
        query_s.push(t.elapsed().as_secs_f64());
        // recompute the length in f64 from the returned path (the crate sums in f32)
        lengths.push(path.map(|p| {
            std::iter::once(start)
                .chain(p.path.iter().copied())
                .collect::<Vec<_>>()
                .windows(2)
                .map(|w| (w[0].as_dvec2() - w[1].as_dvec2()).length())
                .sum()
        }));
    }

    let output = Output {
        triangulate_s,
        merge_s,
        bake_s,
        polygons: mesh.layers[0].polygons.len(),
        query_s,
        lengths,
    };
    serde_json::to_writer(std::io::stdout().lock(), &output).unwrap();
}
