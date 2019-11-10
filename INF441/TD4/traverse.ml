open Cs

module type Graph = sig
  (* the type of graphs *)
  type t

  module V : Cs.Hashable
  type vertex = V.t

  (* run a fold over the immediately reachable vertices from a given vertex *)
  val fold_succ : (vertex -> 'acc -> 'acc) -> t -> vertex -> 'acc -> 'acc
end

module type Traverse = functor (G : Graph) -> sig
  val traverse : G.t -> G.vertex -> G.vertex list
end

module Dfs : Traverse = functor (G : Graph) -> struct
  module Ht = Hashtbl.Make(G.V)

  let traverse g v =
    let visited = Ht.create 19 in
    let rec spin vs wl =
      match wl with
      | [] -> List.rev vs
      | v :: wl when Ht.mem visited v ->
          spin vs wl
      | v :: wl ->
          Ht.add visited v () ;
          let wl = G.fold_succ (fun v wl -> v :: wl) g v wl in
          spin (v :: vs) wl
    in
    spin [] [v]
end

module Bfs : Traverse = functor (G : Graph) -> struct
  module Ht = Hashtbl.Make(G.V)

  let traverse g v =
    let visited = Ht.create 19 in
    let rec spin vs wl =
      match wl with
      | ([], []) -> List.rev vs
      | ([], wlr) -> spin vs (List.rev wlr, [])
      | (v :: wlf, wlr) when Ht.mem visited v ->
          spin vs (wlf, wlr)
      | (v :: wlf, wlr) ->
          Ht.add visited v () ;
          let wlr = G.fold_succ (fun v wlr -> v :: wlr) g v wlr in
          spin (v :: vs) (wlf, wlr)
    in
    spin [] ([v], [])
end
