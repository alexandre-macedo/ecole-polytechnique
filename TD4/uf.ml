(* union find with ranks and path compression *)

module type Uf = sig
  type set

  (** create a new set *)
  val create : unit -> set

  (** combine two sets *)
  val unite : set -> set -> unit

  (** find an identifier of a set. After a (union s1 s2), it must be the case
      that (find s1) = (find s2). *)
  val id : set -> int
end

module Uf : Uf = struct
  type set_contents =
    | Ranked of int * int
    | Ptr of set
  and set = set_contents ref

  let rec find ss =
    match !ss with
    | Ptr p ->
        let pss = find p in
        ss := Ptr pss ;
        pss
    | Ranked _ -> ss

  let __max_id = ref 0
  let create () = incr __max_id ; ref (Ranked (!__max_id, 0))

  let unite ss1 ss2 =
    let ss1 = find ss1 in
    let ss2 = find ss2 in
    if ss1 == ss2 then () else
    match !ss1, !ss2 with
    | _, Ptr _ | Ptr _, _ -> assert false
    | Ranked (x1, r1), Ranked (x2, r2) ->
        if r1 < r2 then begin
          ss2 := Ptr ss1 ;
        end else if r1 = r2 then begin
          ss1 := Ranked (x1, r1 + 1) ;
          ss2 := Ptr ss1 ;
        end else begin
          ss1 := Ptr ss2 ;
        end

  let id ss =
    match !(find ss) with
    | Ptr _ -> assert false
    | Ranked (k, _) -> k

end

include Uf
