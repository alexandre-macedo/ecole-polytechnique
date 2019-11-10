open Cs
open Graph

module Make (V : Vertex) (E : Edge with type vertex = V.t) :
sig
  module VMap : Map.S with type key = V.t
  module VSet : Set.S with type elt = V.t
  include Graph with module V = V with module E = E with type t = E.t list VMap.t
end = struct
  module V = V
  type vertex = V.t

  module E = E
  type edge = E.t

  module VMap = Map.Make (V)
  module VSet = Set.Make (V)
  type t = edge list VMap.t

  let is_empty g = VMap.is_empty g

  let cardinality_v g = VMap.cardinal g

  let cardinality_e g =
    VMap.fold begin fun _ es acc ->
      acc + List.length es
    end g 0

  let out_degree g v =
    VMap.find v g |> List.length

  let iter_v fn g =
    VMap.iter (fun v _ -> fn v) g

  let iter_e fn g =
    VMap.iter (fun _ edgs -> List.iter fn edgs) g

  let in_degree g v =
    let deg = ref 0 in
    iter_e (fun edg -> if V.equal v (E.dest edg) then incr deg) g ;
    !deg

  let mem_v g v = VMap.mem v g

  exception True

  let mem_e g (e : edge) =
    try iter_e (fun ee -> if E.compare e ee = 0 then raise True) g ; false with
    | True -> true

  let is_adj g (u : vertex) (v : vertex) =
    VMap.find u g |>
    List.exists (fun e -> V.equal v (E.dest e))

  let fold_v fn g acc = VMap.fold (fun v _ acc -> fn v acc) g acc

  let fold_e fn g acc =
    VMap.fold begin fun v edgs acc ->
      List.fold_right fn edgs acc
    end g acc

  let fold_succ fn g v acc =
    let edgs = VMap.find v g in
    let next = List.fold_left (fun next e -> VSet.add (E.dest e) next) VSet.empty edgs in
    VSet.fold fn next acc

  let fold_succ_e fn g v acc =
    let edgs = VMap.find v g in
    List.fold_right fn edgs acc

  let fold_pred fn g v acc =
    let vs = fold_e begin fun e vs ->
        if V.equal v (E.dest e) then
          VSet.add (E.source e) vs
        else vs
      end g VSet.empty in
    VSet.fold fn vs acc

  let fold_pred_e fn g v acc =
    let es = fold_e begin fun e es ->
        if V.equal v (E.dest e) then e :: es else es
      end g [] in
    List.fold_right fn es acc

  let iter_succ fn g v = fold_succ (fun v () -> ignore (fn v)) g v ()
  let iter_succ_e fn g v = fold_succ_e (fun e () -> ignore (fn e)) g v ()
  let iter_pred fn g v = fold_pred (fun v () -> ignore (fn v)) g v ()
  let iter_pred_e fn g v = fold_pred_e (fun e () -> ignore (fn e)) g v ()

end

module IntVertex = struct
  type label = int
  type t = label
  let compare (x : t) (y : t) =
    if x < y then -1 else
    if x = y then 0 else 1
  let equal (x : t) y = x = y
  let hash (x : t) = Hashtbl.hash x
end

module IntEdge = struct
  type label = int
  type vertex = IntVertex.t
  type t = {
    source : vertex ;
    dest : vertex ;
    label : label ;
  }
  let compare (x : t) (y : t) =
    if x.label < y.label then -1 else
    if x.label = y.label then 0 else 1
  let source e = e.source
  let dest e = e.dest
  let label e = e.label
  let edge source label dest = {source ; label ; dest}
end

module G = Make (IntVertex) (IntEdge)

module DfsG = Traverse.Dfs (G)
module BfsG = Traverse.Bfs (G)
module ShortestG = Paths.Shortest (G)
module KruskalG = Spanning_tree.Kruskal (G)
