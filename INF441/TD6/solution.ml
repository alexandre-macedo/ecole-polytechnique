(* Partie 1 *)

(* 1.1 *)

type 'a t =
  | Node of 'a t * 'a * 'a t
  | Empty

type 'a tree = 'a t

(* 1.2 *)

(* la fonction leaf ci-dessous simplifie la définition de a1 *)
let leaf x = Node (Empty, x, Empty)

let a1 =
  Node
    (Node (Empty, 3, leaf 5),
     2,
     Node (leaf 8, 6, Node (leaf 15, 12, Empty)))

(* 1.3 *)

(* on commence par quelque chose de simple *)
let rec elements t = match t with
  | Node (t1, x, t2) -> x :: elements t1 @ elements t2
  | Empty -> []

(* 1.4 *)

(* la fonction ci-dessus est inefficace, car elle utilise la concaténation
   de listes, qui coûte O(length (elements t1))

   une meilleure solution, ci-dessous, accumule les éléments en tête d'une
   liste acc (un accumulateur) *)
let rec elements_append t acc =
  match t with
  | Node (t1, x, t2) -> x :: elements_append t1 (elements_append t2 acc)
  | Empty -> acc

(* il suffit alors de l'appeler avec un accumulateur vide *)
let elements t = elements_append t []

let () =
  assert (elements a1 = [2; 3; 5; 6; 8; 12; 15])

(* 1.5 *)

let rec fold f x t = match t with
  | Node (t1, y, t2) ->
     let x = f x y in
     let x = fold f x t1 in
     fold f x t2
  | Empty -> x

(* 1.6 *)

let size t = fold (fun n _ -> n + 1) 0 t

let () =
  assert (size a1 = 7)

(* 1.7 *)

(* c'est le même principe que la (seconde) fonction elements ci-dessus,
   à savoir accumuler en tête de liste, mais il faut penser à renverser
   la liste obtenue au final

   List.rev a une complexité linéaire, donc cette solution reste linéaire *)

let elements t =
  List.rev (fold (fun l x -> x :: l) [] t)

let () =
  assert (elements a1 = [2; 3; 5; 6; 8; 12; 15])

(* Partie 2 *)

(* 2.1 *)

(* c'est tout simplement la comparaison des listes d'éléments,
   en utilisant la comparaison structurelle d'OCaml (=) *)
let equal t1 t2 =
  elements t1 = elements t2

let a2 = Node (leaf 3, 2, Empty)
let a3 = Node (Empty, 2, leaf 3)

let () =
  assert (equal a1 a1);
  assert (equal a2 a3);
  assert (not (equal a1 a2))

(* Partie 3 *)

type 'a iterator = 'a t list

(* 3.1 *)

let iterator : 'a t -> 'a iterator = fun t -> [t]

(* 3.2 *)

let rec next : 'a iterator -> ('a * 'a iterator) option = fun i -> match i with
  | Node (t1, x, t2) :: l -> Some (x, t1 :: t2 :: l)
  | Empty :: l -> next l
  | [] -> None

(* note : on peut optimiser un peu en évitant d'ajouter des arbres vides
   dans la liste lorsque t1 ou t2 est Empty *)
let rec next : 'a iterator -> ('a * 'a iterator) option = fun i -> match i with
  | Node (Empty, x, Empty) :: l -> Some (x, l)
  | (Node (Empty, x, t1) | Node (t1, x, Empty)) :: l -> Some (x, t1 :: l)
  | Node (t1, x, t2) :: l -> Some (x, t1 :: t2 :: l)
  | Empty :: l -> next l
  | [] -> None

(* on pourrait alors faire l'économie de l'avant-dernier cas, mais il
   faudrait dans ce cas redéfinir iterator ainsi : *)
let iterator t =
  match t with
  | Empty -> []
  | t     -> [t]

(* 3.3 *)

let elements t =
  let rec aux i =
    match next i with
    | Some (x, i) -> x :: aux i
    | None -> []
  in
  aux (iterator t)

let () =
  assert (elements Empty = []);
  assert (elements a1 = [2; 3; 5; 6; 8; 12; 15]);
  assert (elements a2 = [2; 3]);
  assert (elements a3 = [2; 3])

(* 3.4 *)

let equal t1 t2 =
  let rec aux i1 i2 =
    match next i1, next i2 with
    | Some (x1, i1), Some (x2, i2) -> x1 = x2 && aux i1 i2
    | None, None -> true
    | _, _ -> false
  in
  aux (iterator t1) (iterator t2)

(* note : le caractère paresseux de && assure bien que le double parcours
   est interrompu aussi tôt que possible *)

let () =
  assert (equal a1 a1);
  assert (equal a2 a3);
  assert (not (equal a1 a2))

(* Partie 4 *)

(* 4.1 [3; 5; 2; 8; 6; 15; 12] *)

(* 4.2 *)

(* ce type représente la branche gauche de l'arbre, de bas en haut,
   où chaque élément est accompagné de son sous-arbre droit *)
type 'a iterator_infix = ('a * 'a tree) list

(* 4.3 *)

(* consiste à descendre le long de la branche gauche, jusqu'à trouver Empty *)
let rec iterator_append t acc =
  match t with
  | Node (tl, x, tr) -> iterator_append tl ((x, tr) :: acc)
  | Empty -> acc

let iterator : 'a tree -> 'a iterator_infix = fun t ->
  iterator_append t []

(* 4.4 *)

let rec next : 'a iterator_infix -> ('a * 'a iterator_infix) option = fun l ->
  match l with
  | (x, t) :: l -> Some (x, iterator_append t l)
  | [] -> None

(* 4.5 *)

(* c'est le même code que 3.3 *)
let elements t =
  let rec aux i =
    match next i with
    | Some (x, i) -> x :: aux i
    | None -> []
  in
  aux (iterator t)

let () =
  assert (elements a1 = [3; 5; 2; 8; 6; 15; 12])

(* Questions bonus *)

(* Partie 5 *)

(* 5.1 *)

module type EqualityType =
  sig
    type t
    val equal : t -> t -> bool
  end

(* 5.2 *)

module Tree(ET : EqualityType) =
  struct
    type element = ET.t

    type t = element tree

    type iterator = t list

    let iterator : t -> iterator = fun t -> [t]

    let rec next : iterator -> (element * iterator) option =
      fun i -> match i with
      | (Node (t1, x, t2)) :: l -> Some (x, t1 :: t2 ::l)
      | Empty :: l -> next l
      | [] -> None

    let equal t1 t2 =
      let rec aux i1 i2 =
        match next i1, next i2 with
        | Some (x1, i1'), Some (x2, i2') -> ET.equal x1 x2 && aux i1' i2'
        | None, None -> true
        | _ -> false
      in
      aux (iterator t1) (iterator t2)
  end

(* 5.3 *)

module O = struct
  type t = int
  let equal = ( = )
end
module T = Tree(O)
module TT = Tree(T)

let () =
  let a23 = Node (Empty, a2, Node (Empty, a3, Empty)) in
  let a32 = Node (Empty, a3, Node (Empty, a2, Empty)) in
  let a32' = Node (Node (Empty, a3, Empty), a2, Empty) in
  assert (TT.equal a23 a32);
  assert (TT.equal a23 a32')

(* Partie 6 *)

(* type 'a iterator = unit -> ('a * 'a iterator) option *)

type 'a iterator_cps =
  | I of (unit -> ('a * 'a iterator_cps) option)

(* 6.1 *)

let next (I i) = i ()

(* 6.2 *)

let empty = I (fun () -> None)

let iterator (t : 'a tree) : 'a iterator_cps =
  let rec aux t k =
    match t with
    | Node (t1,x,t2) -> I (fun () -> Some (x, aux t1 (aux t2 k)))
    | Empty -> k
  in
  aux t empty

(* 6.3 *)

(* c'est toujours le même code (3.3) *)
let elements t =
  let rec aux i =
    match next i with
    | Some (x, i) -> x :: aux i
    | None -> []
  in
  aux (iterator t)

let () =
  assert (elements a1 = [2; 3; 5; 6; 8; 12; 15])

(* 6.4 *)

(* là encore, c'est le même code que 3.4 *)
let equal t1 t2 =
  let rec aux i1 i2 =
    match next i1, next i2 with
    | Some (x1, i1), Some (x2, i2) -> x1 = x2 && aux i1 i2
    | Some _, None -> false
    | None, _ -> true
  in
  aux (iterator t1) (iterator t2)

let () =
  assert (equal a1 a1);
  assert (equal a2 a3);
  assert (not (equal a1 a2))

(* Partie 7 *)

(* gauche / droite / milieu *)
type 'a et = Element of 'a | Tree of 'a tree

type 'a iterator_postfix = 'a et list

let iterator_postfix : 'a tree -> 'a iterator_postfix = fun t -> [Tree t]

let rec next_postfix : 'a iterator_postfix -> ('a * 'a iterator_postfix) option = fun i ->
  match i with
  | Element x :: i -> Some (x, i)
  | Tree Empty :: i -> next_postfix i
  | Tree (Node (tl, x, tr)) :: i ->
      next_postfix (Tree tl :: Tree tr :: Element x :: i)
  | [] -> None

(* note : là encore, pour pourrait optimiser pour éviter d'ajouter des
   Tree Empty dans la liste lorsque tl ou tr est vide (cf 3.2 plus haut) *)

let elements t =
  let rec aux i =
    match next_postfix i with
    | Some (x, i) -> x :: aux i
    | None -> []
  in
  aux (iterator_postfix t)

let () =
  assert (elements a1 = [5; 3; 8; 15; 12; 6; 2])

(* Partie 8 *)

(* 8.1 *)

module Queue = struct
  type 'a t = 'a list
  let empty = []
  let push : 'a t -> 'a -> 'a t= fun q x -> q @ [x] (* inefficace *)
  let pop : 'a t -> ('a * 'a t) option = fun q -> match q with
    | x :: q -> Some (x, q)
    | [] -> None
end

let iterator_width : 'a t -> 'a iterator_cps = fun t ->
  let q = Queue.push Queue.empty t in
  let rec aux q : 'a iterator_cps =
    match Queue.pop q with
    | Some (t, q) ->
       (
         match t with
         | Node (tl, x, tr) ->
            let q = Queue.push q tl in
            let q = Queue.push q tr in
            I (fun () -> Some (x, aux q))
         | Empty -> aux q
       )
    | None -> I (fun () -> None)
  in
  aux q

let elements_width t =
  let rec aux i =
    match next i with
    | Some (x, i) -> x :: aux i
    | None -> []
  in
  aux (iterator_width t)

let () =
  assert (elements_width a1 = [2; 3; 6; 5; 8; 12; 15])
