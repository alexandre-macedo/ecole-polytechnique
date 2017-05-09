(* -------------------------------------------------------------------- *)
let pmap f g = fun (x, y) -> (f x, g y)

(*
let pmap_type :
  'a1 'a2 'b1 'b2.
    ('a1 -> 'a2) -> ('b1 -> 'b2) -> ('a1 * 'b1) -> ('a2 * 'b2)
= pmap
*)

(* -------------------------------------------------------------------- *)
let idc (x : int) = ((fun y -> y + x), (fun y -> y - x))

(* -------------------------------------------------------------------- *)
let comp f g = fun x -> f (g x)

(*
let comp_type :
  'a 'b 'c. ('b -> 'c) -> ('a -> 'b) -> 'a -> 'c
= comp
*)

(* -------------------------------------------------------------------- *)
let _ =
  let fid = (let (ic, dc) = idc 10 in comp ic dc) in
  fun x -> assert (fid x = x)

(* -------------------------------------------------------------------- *)
let rec fact (x : int) =
  if x <= 0 then 1 else x * (fact (x-1))

(* -------------------------------------------------------------------- *)
let rec fact_r (r : int) (x : int) =
  if x <= 0 then r else fact_r (r * x) (x - 1)

let fact' = fact_r 1

(* -------------------------------------------------------------------- *)
let rec exp_naive (x : int) (n : int) =
  if n <= 0 then 1 else x * (exp_naive x (n - 1))

let exp (x : int) =
  let rec aux (r : int) (n : int) =
    if n <= 0 then r else aux (r * x) (n - 1)
  in fun n -> aux 1 n

let fast_exp (x : int) =
  let rec aux (n : int) =
    if n <= 0 then 1 else
    let r = aux (n/2) in
    if n land 1 = 0 then r * r else r + r * r
  in fun n -> aux n

(* -------------------------------------------------------------------- *)
let rec even = function
  | 0 -> true
  | n when n < 0 -> even (-n)
  | n -> odd (n-1)

and odd = function
  | 0 -> false
  | n when n < 0 -> odd (-n)
  | n -> even (n-1)

let rec parity (b : bool) = function
  | 0 -> b
  | n when n < 0 -> parity b (-n)
  | n -> parity (not b) (n-1)

let parity' = function true -> even | false -> odd

(* -------------------------------------------------------------------- *)
let rec fibo (n : int) =
  if n <= 1 then 1 else fibo (n-1) + fibo (n-2)

(* -------------------------------------------------------------------- *)
let rec fibo_r (a : int) (b : int) (n : int) =
  if n <= 0 then a else fibo_r b (a+b) (n-1)

let fibo' = fibo_r 1 1

(* -------------------------------------------------------------------- *)
let rec sum = function [] -> 0 | x::s -> x + sum s

(* -------------------------------------------------------------------- *)
let gsum f =
  let rec aux =
    function [] -> 0 | x::s -> f x + aux s
  in fun s -> aux s

let sum' = gsum (fun x -> x)

(* -------------------------------------------------------------------- *)
let rec zfilter = function
  | [] -> []
  | x::s when x < 0 -> zfilter s
  | x::s -> x :: zfilter s

(* -------------------------------------------------------------------- *)
let filter (p : 'a -> bool) =
  let rec aux = function
    | [] -> []
    | x::s when p x -> x :: aux s
    | _::s -> aux s
  in fun s -> aux s

(*
let filter_type :
  'a. ('a -> bool) -> 'a list -> 'a list
= filter
*)

(* -------------------------------------------------------------------- *)
let zfilter' = filter (fun x -> 0 <= x)

(* -------------------------------------------------------------------- *)
let insert x =
  let rec aux = function
    | [] -> [x]
    | (y :: _) as s when x <= y -> x :: s
    | (y :: s) -> y :: aux s
  in fun s -> aux s

(* -------------------------------------------------------------------- *)
let rec sort = function [] -> [] | x::s -> insert x (sort s)

(* -------------------------------------------------------------------- *)
let ginsert (rel : 'a -> 'a -> bool) (x : 'a) =
  let rec aux = function
    | [] -> [x]
    | (y :: _) as s when rel x y -> x :: s
    | (y :: s) -> y :: aux s
  in fun s -> aux s

(* -------------------------------------------------------------------- *)
let gsort (rel : 'a -> 'a -> bool) =
  let rec aux =
    function [] -> [] | x::s -> ginsert x rel (aux s)
  in fun s -> aux s

(* -------------------------------------------------------------------- *)
let gsorted (rel : 'a -> 'a -> bool) =
  let rec aux = function
    | [] | [_] -> true
    | x :: ((y :: _) as s) -> rel x y && aux s
  in fun s -> aux s

(* -------------------------------------------------------------------- *)
let map (f : 'a -> 'b) =
  let rec aux =
    function [] -> [] | x :: s -> f x :: aux s
  in fun s -> aux s

let rec flatten = function [] -> [] | x :: s -> x @ flatten s

let rec rev_append (s1 : 'a list) (s2 : 'a list) =
  match s1 with [] -> s2 | x :: s1 -> rev_append s1 (x :: s2)

let insert_at_all_positions (x : int) =
  let rec aux acc hd tl =
    let acc = (rev_append hd (x :: tl)) :: acc in
    match tl with [] -> List.rev acc | x :: tl -> aux acc (x :: hd) tl
  in fun s -> aux [] [] s

let rec perm = function
  | [] -> [[]]
  | x :: s -> flatten (map (insert_at_all_positions x) (perm s))

(* -------------------------------------------------------------------- *)
type complex = { re : float; im: float; }

let add x y =
  { re = x.re +. y.re; im = x.im +. y.im; }

let opp x =
  { re = -. x.re; im = -. x.im; }

let mul x y =
  { re = x.re *. y.re -. x.im *. y.im;
    im = x.re *. y.im +. x.im *. y.re; }

let inv x =
  let d = x.re *. x.re +. x.im *. x.im in
  { re = x.re /. d; im = -. x.re /. d; }


(* -------------------------------------------------------------------- *)
type suit   = Heart | Tile | Clover | Pike
type cvalue = As | Plain of int | Jack | Queen | King
type card   = suit * cvalue

let rec range i j =
  if j < i then [] else i :: range (i+1) j

let all_values =
  As :: (map (fun x -> Plain x) (range 2 10)) @ [Jack; Queen; King]

let all_suits =
  [Heart; Tile; Clover; Pike]

let all_cards_52 =
  flatten (map (fun s -> map (fun c -> (s, c)) all_values) all_suits)

let all_cards_32 =
  let p (_, c) =
    match c with
    | Plain i -> 7 <= i && i <= 10
    | _ -> true
  in filter p all_cards_52

let belote_1 (atout : bool) (c : cvalue) =
  if atout then
    match c with
    | As       -> 11
    | Plain 9  -> 14
    | Plain 10 -> 10
    | Plain _  -> 0
    | Jack     -> 20
    | Queen    -> 3
    | King     -> 4
  else
    match c with
    | As       -> 11
    | Plain 10 -> 10
    | Plain _  -> 0
    | Jack     -> 2
    | Queen    -> 3
    | King     -> 4

let belote (s : suit) (deck : card list) =
  let score (s', c) = belote_1 (s = s') c in
  sum (map score deck)

(* -------------------------------------------------------------------- *)
let counter (x : int) =
  let r = ref 0 in fun () -> let aout = !r in (r := !r + x; aout)