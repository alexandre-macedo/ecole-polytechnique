open Cs

module type Weight = sig
  type t
  val to_int : t -> int
end

module type Graph = sig
  type t

  module V : Cs.Hashable
  type vertex = V.t

  module E : sig
    type t
    val dest   : t -> vertex
  end
  type edge = E.t

  (* run a fold over the edges that go out of a given vertex *)
  val fold_succ_e : (edge -> 'acc -> 'acc) -> t -> vertex -> 'acc -> 'acc
end

module type Paths = functor (G : Graph) (Wt : Weight with type t = G.E.t) -> sig
  (* find g v0 v1 : find a path between v0 and v1 in g *)
  (* raise Not_found if no path exists *)
  val find : G.t -> G.vertex -> G.vertex -> G.edge list
end

module Shortest : Paths = functor (G : Graph) (Wt : Weight with type t = G.E.t) -> struct
  module Ht = Hashtbl.Make (G.V)

  let rec insert ((path, plen, dest) as thing) q =
    match q with
    | ((_, len, _) as old) :: q when len < plen ->
        old :: insert thing q
    | _ -> thing :: q

  let find g src dst =
    let visited = Ht.create 19 in
    let rec spin q =
      match q with
      | [] -> raise Not_found
      | (path, plen, v) :: _ when G.V.equal v dst -> List.rev path
      | (path, plen, v) :: q ->
          Ht.add visited v () ;
          let q = G.fold_succ_e begin fun e q ->
              let vd = G.E.dest e in
              if Ht.mem visited vd then q else
              let delta = Wt.to_int e in
              insert (e :: path, plen + delta, vd) q
            end g v q in
          spin q
    in
    spin [[], 0, src]
end
