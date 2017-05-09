open Cs
open Spanning_tree

module Graph = struct
  module V = struct
    type t = int
    let equal (x : t) y = x = y
    let hash (x : t) = Hashtbl.hash x
  end
  type vertex = V.t

  module E = struct
    type t = vertex * vertex

    let source (s, d) = s
    let dest (s, d) = d
  end
  type edge = E.t

  type t = (vertex * edge list) list

  let rec iter_v fn g =
    match g with
    | [] -> ()
    | (v, _) :: g -> fn v ; iter_v fn g

  let rec fold_e fn g acc =
    match g with
    | [] -> acc
    | (v, edgs) :: g ->
        let acc = List.fold_right fn edgs acc in
        fold_e fn g acc
end

module KruskalTest = Kruskal (Graph)

let example : Graph.t =
  [ 0, [(0, 1) ; (0, 2)] ;
    1, [(1, 2) ; (1, 5)] ;
    2, [(2, 0) ; (2, 3) ; (2, 4)] ;
    3, [(3, 1)] ;
    4, [] ;
    5, [(5, 0)] ;
    6, [(6, 8)] ;
    7, [(7, 8)] ;
    8, [(8, 7) ; (8, 9)] ;
    9, [] ]
