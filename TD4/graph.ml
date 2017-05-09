open Cs

module type Vertex = sig
  type t
  (* all vertices are comparable *)
  include Comparable with type t := t
end

module type Edge = sig
  type t

  (* all edges are ordered *)
  include Ordered with type t := t

  type vertex
  val source : t -> vertex
  val dest   : t -> vertex

  type label
  val label  : t -> label

  (** edge src lab dst : edge with source src, dest dst, label lab *)
  val edge : vertex -> label -> vertex -> t
end

module type Graph = sig
  type t

  module V : Vertex
  type vertex = V.t

  module E : Edge with type vertex = V.t
  type edge = E.t

  val is_empty : t -> bool
  val cardinality_v : t -> int (* how many vertices? *)
  val cardinality_e : t -> int (* how many edges? *)

  (* Both the following raise Invalid_argument if
     the vertex is not in the graph *)
  val in_degree : t -> vertex -> int
  val out_degree : t -> vertex -> int

  (** mem_v g v : is v in g? *)
  val mem_v : t -> vertex -> bool

  (** mem_e g e : is e in g? *)
  val mem_e : t -> edge -> bool

  (** is_adj g v1 v2 : is v2 reachable with one edge from v1? *)
  val is_adj : t -> vertex -> vertex -> bool

  (* folds *)

  (** fold_v f g acc = fold f over all vertices in g using acc *)
  val fold_v : (vertex -> 'acc -> 'acc) -> t -> 'acc -> 'acc

  (** fold_e f g acc = fold f over all edges in g using acc *)
  val fold_e : (edge -> 'acc -> 'acc) -> t -> 'acc -> 'acc

  (** fold_succ f g v acc = fold f over the successor vertices of v in g using acc *)
  val fold_succ : (vertex -> 'acc -> 'acc) -> t -> vertex -> 'acc -> 'acc

  (** fold_succ_e f g v acc = fold f over the out edges of v in g using acc *)
  val fold_succ_e : (edge -> 'acc -> 'acc) -> t -> vertex -> 'acc -> 'acc

  (** fold_pred f g v acc = fold f over the predecessor vertices of v in g using acc *)
  val fold_pred : (vertex -> 'acc -> 'acc) -> t -> vertex -> 'acc -> 'acc

  (** fold_pred_e f g v acc = fold f over the in edges of v in g using acc *)
  val fold_pred_e : (edge -> 'acc -> 'acc) -> t -> vertex -> 'acc -> 'acc

  (* iterators *)

  val iter_v : (vertex -> unit) -> t -> unit
  val iter_e : (edge -> unit) -> t -> unit
  val iter_succ : (vertex -> unit) -> t -> vertex -> unit
  val iter_succ_e : (edge -> unit) -> t -> vertex -> unit
  val iter_pred : (vertex -> unit) -> t -> vertex -> unit
  val iter_pred_e : (edge -> unit) -> t -> vertex -> unit
end
