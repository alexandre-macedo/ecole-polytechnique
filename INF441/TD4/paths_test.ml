open Cs

module Graph = struct
   module V = struct
     type t = int
     let equal (x : t) y = x = y
     let hash (x : t) = Hashtbl.hash x
   end
   type vertex = V.t

   module E = struct
     type t = int * vertex

     let dest (el, ed) = ed
   end
   type edge = E.t

   type t = (vertex * edge list) list

   let fold_succ_e fn g v acc =
     let rec spin als =
       match als with
       | [] -> raise Not_found
       | (u, edgs) :: _ when u = v ->
           List.fold_right fn edgs acc
       | _ :: als ->
           spin als
     in
     spin g
end

module Wt = struct
  type t = Graph.E.t
  let to_int (el, ed) = el
end

module ShortestTest = Paths.Shortest (Graph) (Wt)

let example : Graph.t =
  [ 0, [ (2, 2) ; (4, 1) ] ;
    1, [ (5, 2) ; (10, 3) ] ;
    2, [ (3, 4) ] ;
    3, [ (11, 5) ] ;
    4, [ (4, 3) ] ;
    5, [] ]
