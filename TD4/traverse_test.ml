open Cs
open Traverse

module Graph = struct
   module V = struct
     type t = int
     let equal (x : t) y = x = y
     let hash (x : t) = Hashtbl.hash x
   end
   type vertex = V.t

   type t = (vertex * vertex list) list

   let fold_succ fn g v acc =
     let rec spin als =
       match als with
       | [] -> raise Not_found
       | (u, usuccs) :: _ when u = v ->
           List.fold_right fn usuccs acc
       | _ :: als ->
           spin als
     in
     spin g
end

module DfsTest = Dfs(Graph)
module BfsTest = Bfs(Graph)


let example : Graph.t =
  [ 0, [1 ; 2] ;
    1, [2 ; 5] ;
    2, [0 ; 3 ; 4] ;
    3, [1] ;
    4, [] ;
    5, [0] ]
