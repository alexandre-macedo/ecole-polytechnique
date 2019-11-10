open Cs

module type Graph = sig
  type t

  module V : Cs.Hashable
  type vertex = V.t

  module E : sig
    type t

    val source : t -> vertex
    val dest : t -> vertex
  end
  type edge = E.t

  val iter_v : (vertex -> unit) -> t -> unit

  val fold_e : (edge -> 'acc -> 'acc) -> t -> 'acc -> 'acc
end

module type Tree = functor (G : Graph) -> sig
  val spanning : G.t -> G.edge list
end

module Kruskal : Tree = functor (G : Graph) ->
struct
  module Ht = Hashtbl.Make (G.V)

  let spanning g =
    let sets = Ht.create 19 in
    G.iter_v (fun v -> Ht.add sets v (Uf.create ())) g ;
    G.fold_e begin fun edg spt ->
      let src = G.E.source edg in
      let dst = G.E.dest edg in
      let src_set = Ht.find sets src in
      let dst_set = Ht.find sets dst in
      if Uf.id src_set = Uf.id dst_set then spt else begin
        Uf.unite src_set dst_set ;
        edg :: spt
      end
    end g []
end
