open Cs
open Graph

module type Digraph = sig
  include Graph

  val empty : t

  val add_vertex : t -> vertex -> t
  val add_edge   : t -> edge -> t

  val remove_vertex : t -> vertex -> t
  val remove_edge   : t -> edge -> t
end

module Digraph (V : Vertex) (E : Edge with type vertex = V.t)
: Digraph with module V = V with module E = E =
struct
  include Graph_test.Make (V) (E)

  let empty = VMap.empty

  let add_vertex g v = VMap.add v [] g
  let add_edge g e =
    let v = E.source e in
    let edgs = VMap.find v g in
    VMap.add v (e :: edgs) g

  let remove_edge g e =
    let v = E.source e in
    let edgs = VMap.find v g in
    let edgs = List.filter (fun ee -> E.compare e ee <> 0) edgs in
    VMap.add v edgs g

  let remove_vertex g v =
    VMap.fold begin fun u es g ->
      if V.equal u v then g else
      let es = List.filter begin fun e ->
          not (V.equal (E.source e) v || V.equal (E.dest e) v)
        end es in
      if es = [] then g else VMap.add u es g
    end g empty

end

module type Builders =
  functor (G : Digraph with type V.t = int
                       with type E.label = int) ->
sig
  val vertices : int -> G.t
  val full : int -> G.t
  val divisors : int -> G.t
end

module Builders : Builders =
  functor (G : Digraph with type V.t = int
                       with type E.label = int) ->
struct
  let vertices n =
    let rec spin g k =
      if k < 0 then g else
      let g = G.add_vertex g k in
      spin g (k - 1)
    in
    spin G.empty (n - 1)

  let full n =
    let g = vertices n in
    let rec spin g i j =
      if i < 0 then g else
      if j < 0 then spin g (i - 1) (n - 1) else
      let g = G.add_edge g (G.E.edge i (j * n + i) j) in
      spin g i (j - 1)
    in
    spin g (n - 1) (n - 1)

  let divisors n =
    let g = vertices n in
    let rec spin g i j =
      if i < 2 then g else
      if j < 2 then spin g (i - 1) (n + 1) else
      let g = if j mod i = 0 then G.add_edge g (G.E.edge i (j * n + i) j) else g in
      spin g i (j - 1)
    in
    spin g (n + 1) (n + 1)
end
