type ident = string

type intex =
  | Const of int
  | Var   of ident
  | Add   of intex * intex
  | Sub   of intex * intex
  | Times of intex * intex
  | Neg   of intex

type pred = Eq | Lt | Leq | Gt | Geq | Neq | Div | Ndiv

type boolex =
  | Pred   of pred * intex * intex
  | And    of boolex * boolex | True
  | Or     of boolex * boolex | False
  | Impl   of boolex * boolex
  | Not    of boolex
  | Forall of string * boolex
  | Exists of string * boolex

(* Here are some test expressions *)
module E = struct
  let ante = Pred (Geq, Var "u", Const 5)
  let succbod = Pred (Eq, Var "u",
                      Add (Times (Const 2, Var "y"), Times (Const 3, Var "z")))
  let succ = Exists ("y", Exists ("z", succbod))
  let i = Impl (ante, succ)
  let t = Forall ("u", i)

  let e =
    And (Pred (Lt, Times (Const 2, Var "x"), Add (Var "z", Const 6)),
         And (Pred (Lt, Sub (Var "y", Const 1), Times (Const 3, Var "x")),
              Pred (Div, Const 4, Add (Times (Const 5, Var "x"), Const 1))))

  let f =
    And (Or (Pred (Lt, Add (Times (Const 3, Var "x"), Const 1), Const 10),
             Pred (Gt, Sub (Times (Const 7, Var "x"), Const 6), Const 7)),
         Pred (Div, Const 2, Var "x"))

end

(******************************************************************************)

open Print_tree

let rec intex_to_node e =
  match e with
  | Const k -> exact (string_of_int k)
  | Var x -> exact x
  | Add (e1, e2) ->
      binop 6 true " + " (intex_to_node e1) (intex_to_node e2)
  | Sub (e1, e2) ->
      binop 7 false " - " (intex_to_node e1) (intex_to_node e2)
  | Times (e1, e2) ->
      binop 8 true " * " (intex_to_node e1) (intex_to_node e2)
  | Neg e ->
      unop 9 "~ " (intex_to_node e)

let rec boolex_to_node e =
  match e with
  | Pred (p, e1, e2) -> begin
      let pe1 = intex_to_node e1 in
      let pe2 = intex_to_node e2 in
      binop 4 false begin
        match p with
        | Eq  -> " = "
        | Neq -> " != "
        | Lt  -> " < "
        | Leq -> " <= "
        | Gt  -> " > "
        | Geq -> " >= "
        | Div -> " : "
        | Ndiv -> " % "
      end pe1 pe2
    end
  | And (e1, e2) ->
      binop 3 true " & "
        (boolex_to_node e1)
        (boolex_to_node e2)
  | True -> exact "true"
  | False -> exact "false"
  | Or (e1, e2) ->
      binop 2 true " | "
        (boolex_to_node e1)
        (boolex_to_node e2)
  | Impl (e1, e2) ->
      binop 1 true " => "
        (boolex_to_node e1)
        (boolex_to_node e2)
  | Not e ->
      unop 5 "! "
        (boolex_to_node e)
  | Forall (x, e) ->
      unop 0 ("\\A " ^ x ^ ". ")
        (boolex_to_node e)
  | Exists (x, e) ->
      unop 0 ("\\E " ^ x ^ ". ")
        (boolex_to_node e)

let intex_to_string e = (intex_to_node e)#to_string
let boolex_to_string e = (boolex_to_node e)#to_string

(******************************************************************************)

class ['acc] traverse = object (self)
  method intex (acc : 'acc) (e0 : intex) =
    match e0 with
    | Const _ | Var _ -> acc
    | Add (e1, e2) | Sub (e1, e2) | Times (e1, e2) ->
        let acc = self#intex acc e1 in
        self#intex acc e2
    | Neg e -> self#intex acc e

  method boolex acc e0 =
    match e0 with
    | Pred (p, e1, e2) ->
        let acc = self#intex acc e1 in
        self#intex acc e2
    | And (e1, e2) | Or (e1, e2) ->
        let acc = self#boolex acc e1 in
        self#boolex acc e2
    | True | False -> acc
    | Impl (e1, e2) ->
        let acc = self#boolex acc e1 in
        self#boolex acc e2
    | Not e ->
        self#boolex acc e
    | Forall (x, e) | Exists (x, e) ->
        self#boolex acc e
end

exception Is_free

let check_nonfree x = object (self)
  inherit [unit] traverse as super

  method! intex () e =
    match e with
    | Var y when x = y -> raise Is_free
    | _ -> super#intex () e

  method! boolex () e =
    match e with
    | ( Forall (y, _) | Exists (y, _) )
      when x = y -> ()
    | _ -> super#boolex () e
end

let is_nonfree = object (self)
  inherit [ident] traverse as super

  method! intex x e =
    match e with
    | Var y when x = y -> raise Is_free
    | _ -> super#intex x e

  method! boolex x e =
    match e with
    | ( Forall (y, _) | Exists (y, _) )
      when x = y -> x
    | _ -> super#boolex x e
end

module Idts : Set.S with type elt := ident =
  Set.Make (struct
    type t = ident
    let compare = String.compare
  end)

let find_vars = object (self)
  inherit [Idts.t] traverse as super

  method! intex vs e =
    match e with
    | Var x -> Idts.add x vs
    | _ -> super#intex vs e

  method! boolex vs e =
    match e with
    | Forall (x, e) | Exists (x, e) ->
        let vs = self#boolex vs e in
        Idts.remove x vs
    | _ -> super#boolex vs e
end

exception Malformed_times of intex

let check_times = object (self)
  inherit [unit] traverse as super

  method! intex () e =
    match e with
    | Times (Const k, _) | Times (_, Const k) -> ()
    | Times _ -> raise (Malformed_times e)
    | _ -> super#intex () e
end

(******************************************************************************)

class ['acc] map = object (self)
  method intex (acc : 'acc) (e : intex) : 'acc * intex =
    match e with
    | Var _ | Const _ -> (acc, e)
    | Add (e1, e2) ->
        let (acc, e1) = self#intex acc e1 in
        let (acc, e2) = self#intex acc e2 in
        let e = match e1, e2 with
          | Const j, Const k -> Const (j + k)
          | _ -> Add (e1, e2)
        in
        (acc, e)
    | Sub (e1, e2) ->
        let (acc, e1) = self#intex acc e1 in
        let (acc, e2) = self#intex acc e2 in
        let e = match e1, e2 with
          | Const j, Const k -> Const (j - k)
          | _ -> Sub (e1, e2)
        in
        (acc, e)
    | Times (e1, e2) ->
        let (acc, e1) = self#intex acc e1 in
        let (acc, e2) = self#intex acc e2 in
        let e = match e1, e2 with
          | Const j, Const k -> Const (j * k)
          | _ -> Times (e1, e2)
        in
        (acc, e)
    | Neg e ->
        let (acc, e) = self#intex acc e in
        let e = match e with
          | Const j -> Const (- j)
          | _ -> Neg e
        in
        (acc, e)

  method boolex (acc : 'acc) (e : boolex) : 'acc * boolex =
    match e with
    | Pred (p, e1, e2) ->
        let (acc, e1) = self#intex acc e1 in
        let (acc, e2) = self#intex acc e2 in
        let e = match e1, e2 with
          | Const j, Const k ->
              if begin match p with
                | Eq -> j = k
                | Lt -> j < k
                | Leq -> j <= k
                | Gt -> j > k
                | Geq -> j >= k
                | Neq -> j <> k
                | Div -> j mod k = 0
                | Ndiv -> j mod k <> 0
              end then True else False
          | _ -> Pred (p, e1, e2)
        in
        (acc, e)
    | And (e1, e2) ->
        let (acc, e1) = self#boolex acc e1 in
        let (acc, e2) = self#boolex acc e2 in
        let e = match e1, e2 with
          | True, True -> True
          | False, True | False, False | True, False -> False
          | _ -> And (e1, e2)
        in
        (acc, e)
    | Or (e1, e2) ->
        let (acc, e1) = self#boolex acc e1 in
        let (acc, e2) = self#boolex acc e2 in
        let e = match e1, e2 with
          | False, False -> False
          | False, True | True, True | True, False -> True
          | _ -> Or (e1, e2)
        in
        (acc, e)
    | True | False -> (acc, e)
    | Impl (e1, e2) ->
        let (acc, e1) = self#boolex acc e1 in
        let (acc, e2) = self#boolex acc e2 in
        let e = match e1, e2 with
          | False, True | True, True | False, False -> True
          | True, False -> False
          | _ -> Impl (e1, e2)
        in
        (acc, e)
    | Not e ->
        let (acc, e) = self#boolex acc e in
        let e = match e with
          | True -> False
          | False -> True
          | _ -> Not e
        in
        (acc, e)
    | Forall (x, e) ->
        let (acc, e) = self#boolex acc e in
        let e = match e with
          | True | False -> e
          | e -> Forall (x, e)
        in
        (acc, e)
    | Exists (x, e) ->
        let (acc, e) = self#boolex acc e in
        let e = match e with
          | True | False -> e
          | e -> Exists (x, e)
        in
        (acc, e)
end

let is_nonfree_boolex x b =
  match is_nonfree#boolex x b with
  | _ -> true
  | exception Is_free -> false

let is_free_boolex x b = not (is_nonfree_boolex x b)

let is_nonfree_intex x e =
  match is_nonfree#intex x e with
  | _ -> true
  | exception Is_free -> false

let miniscope = object (self)
  inherit [unit] map as super

  method! boolex () e =
    match e with
    | Forall (x, e) -> begin
        let ((), e) = self#boolex () e in
        if is_nonfree_boolex x e then ((), e) else
        let e = match e with
          | And (e1, e2) ->
              And (Forall (x, e1), Forall (x, e2))
          | Or (e1, e2) -> begin
              match is_free_boolex x e1, is_free_boolex x e2 with
              | true, true ->
                  Forall (x, e)
              | true, false ->
                  Or (Forall (x, e1), e2)
              | false, true ->
                  Or (e1, Forall (x, e2))
              | false, false ->
                  assert false
            end
          | _ -> Forall (x, e)
        in
        ((), e)
      end
    | Exists (x, e) -> begin
        let ((), e) = self#boolex () e in
        if is_nonfree_boolex x e then ((), e) else
        let e = match e with
          | Or (e1, e2) ->
              Or (Exists (x, e1), Exists (x, e2))
          | And (e1, e2) -> begin
              match is_free_boolex x e1, is_free_boolex x e2 with
              | true, true ->
                  Exists (x, e)
              | true, false ->
                  And (Exists (x, e1), e2)
              | false, true ->
                  And (e1, Exists (x, e2))
              | false, false ->
                  assert false
            end
          | _ -> Exists (x, e)
        in
        ((), e)
      end
    | _ -> super#boolex () e
end

let prop_simplify = object (self)
  inherit [unit] map as super

  method! boolex () e =
    match e with
    | And (e1, e2) -> begin
        let (), e1 = self#boolex () e1 in
        let (), e2 = self#boolex () e2 in
        let e = match e1, e2 with
          | True, e | e, True -> e
          | False, _ | _, False -> False
          | _ -> And (e1, e2)
        in
        ((), e)
      end
    | Or (e1, e2) -> begin
        let (), e1 = self#boolex () e1 in
        let (), e2 = self#boolex () e2 in
        let e = match e1, e2 with
          | False, e | e, False -> e
          | True, _ | _, True -> True
          | _ -> Or (e1, e2)
        in
        ((), e)
      end
    | Impl (e1, e2) ->
        self#boolex () (Or (Not e1, e2))
    | _ -> super#boolex () e

end

let strength_reduce = object (self)
  inherit [unit] map as super

  method! boolex () e =
    match e with
    | Pred (p, e1, e2) ->
        let (), e1 = self#intex () e1 in
        let (), e2 = self#intex () e2 in
        let e = match p with
          | Eq -> And (Pred (Lt, e1, Add (e2, Const 1)),
                       Pred (Lt, e2, Add (e1, Const 1)))
          | Neq -> Or (Pred (Lt, e1, e2),
                       Pred (Lt, e2, e1))
          | Lt -> Pred (Lt, e1, e2)
          | Leq -> Pred (Lt, e1, Add (e2, Const 1))
          | Gt -> Pred (Lt, e2, e1)
          | Geq -> Pred (Lt, e2, Add (e1, Const 1))
          | Div | Ndiv -> Pred (p, e1, e2)
        in
        ((), e)
    | _ -> super#boolex () e

end

(* This shows that inheritence and overriding is not
   always the best thing if you need to do something
   that touches the entire tree. *)
let negation_normalize = object (self)
  inherit [bool] map as super

  method! boolex ispos e =
    match e with
    | Not e -> self#boolex (not ispos) e
    | Impl (e1, e2) -> self#boolex ispos (Or (Not e1, e2))
    | _ ->
        let _, e = super#boolex ispos e in
        let e =
          if ispos then e else
          match e with
          | Pred (Eq, e1, e2) -> Pred (Neq, e1, e2)
          | Pred (Neq, e1, e2) -> Pred (Eq, e1, e2)
          | Pred (Lt, e1, e2) -> Pred (Geq, e1, e2)
          | Pred (Leq, e1, e2) -> Pred (Gt, e1, e2)
          | Pred (Gt, e1, e2) -> Pred (Leq, e1, e2)
          | Pred (Geq, e1, e2) -> Pred (Lt, e1, e2)
          | Pred (Div, e1, e2) -> Pred (Ndiv, e1, e2)
          | Pred (Ndiv, e1, e2) -> Pred (Div, e1, e2)
          | And (e1, e2) -> Or (e1, e2)
          | True -> False
          | Or (e1, e2) -> And (e1, e2)
          | False -> True
          | Exists (x, e) -> Forall (x, e)
          | Forall (x, e) -> Exists (x, e)
          | Not _ | Impl _ -> assert false
        in
        (ispos, e)
end

let update_dirty ?(dirty = false) fn data e0 =
  let (data, e) = fn data e0 in
  let dirty = dirty || e <> e0 in
  (dirty, data, e)

let simplify e0 =
  let rec spin e =
    let (dirty, _, e) = update_dirty negation_normalize#boolex true e in
    let (dirty, _, e) = update_dirty miniscope#boolex () e in
    let (dirty, _, e) = update_dirty prop_simplify#boolex () e in
    let (dirty, _, e) = update_dirty strength_reduce#boolex () e in
    if dirty then spin e else e
  in
  spin e0

(******************************************************************************)

module Idtab : Map.S with type key := ident = Map.Make (String)

let get_coeff tab x =
  try Idtab.find x tab with
  | Not_found -> 0

let map_coeff fn tab x =
  let k = get_coeff tab x in
  match fn k with
  | 0 -> Idtab.remove x tab
  | k -> Idtab.add x k tab

let cvar = "#"

let rec unpack ?(tab = Idtab.empty) ?(fac = 1) e =
  match e with
  | Const k -> map_coeff (fun _ -> k * fac) tab cvar
  | Var x -> map_coeff (fun _ -> fac) tab x
  | Add (e1, e2) ->
      let tab = unpack ~tab ~fac e1 in
      unpack ~tab ~fac e2
  | Sub (e1, e2) ->
      let tab = unpack ~tab ~fac e1 in
      unpack ~tab ~fac:(- fac) e2
  | Neg e ->
      unpack ~tab ~fac:(- fac) e
  | Times (Const k, e)
  | Times (e, Const k) ->
      unpack ~tab ~fac:(fac * k) e
  | Times _ ->
      Printf.sprintf "Cannot unpack: %s" (intex_to_string e)
      |> failwith

let unpack_for x e =
  let tab = unpack e in
  let k = get_coeff tab x in
  (k, Idtab.remove x tab)

let repack tab =
  let k = get_coeff tab cvar in
  let (y, k) =
    if k = 0 && not (Idtab.is_empty tab)
    then Idtab.choose tab
    else (cvar, k)
  in
  let e = if y = cvar then Const k else Times (Const k, Var y) in
  Idtab.fold begin fun x k e ->
    if x = y then e else
    Add (Times (Const k, Var x), e)
  end tab e

let collect_terms = object (self)
  inherit [ident] map as super

  method! boolex x e =
    match simplify e with
    | Pred (Lt, e1, e2) ->
        let (k1, tab1) = unpack_for x e1 in
        let (k2, tab2) = unpack_for x e2 in
        if k1 > k2 then begin
          let tab1 = Idtab.map (fun c -> - c) tab1 in
          let tab = Idtab.union (fun _ k1 k2 -> Some (k2 + k1)) tab1 tab2 in
          (x, Pred (Lt, Times (Const (k1 - k2), Var x), repack tab))
        end else if k2 > k1 then begin
          let tab2 = Idtab.map (fun c -> - c) tab2 in
          let tab = Idtab.union (fun _ k1 k2 -> Some (k1 + k2)) tab1 tab2 in
          (x, Pred (Lt, repack tab, Times (Const (k2 - k1), Var x)))
        end else begin
          (x, Pred (Lt, repack tab1, repack tab2))
        end
    | Pred ((Div | Ndiv as p), e1, e2) ->
        let e1 = repack (unpack e1) in
        let (k2, tab) = unpack_for x e2 in
        let e = if k2 = 0
          then Pred (p, e1, repack tab)
          else Pred (p, e1, Add (Times (Const k2, Var x), repack tab))
        in
        (x, e)
    | e -> super#boolex x e
end

let rec gcd m n = if n = 0 then m else gcd n (m mod n)
let lcm m n = m * n / gcd m n

let get_lcm = object (self)
  inherit [ident * int] traverse as super

  method! intex (x, l) e =
    let (k, tab) = unpack_for x e in
    let l = if k = 0 then l else lcm k l in
    (x, l)
end

let set_lcm = object (self)
  inherit [ident * int] map as super

  method! intex (x, d) e =
    let e = unpack e |>
            Idtab.map (fun c -> c * d) |>
            repack
    in
    ((x, d), e)

  method! boolex (x, delta) e =
    match e with
    | Pred (p, e1, e2) -> begin
        let (k1, tab1) = unpack_for x e1 in
        let (k2, tab2) = unpack_for x e2 in
        match k1, k2 with
        | 0, 0 ->
            ((x, delta), e)
        | 0, k ->
            let d = delta / k in
            let e1 = Idtab.map (fun c -> c * d) tab1 |> repack in
            let e2 = Idtab.map (fun c -> c * d) tab2
                     |> Idtab.add x 1 |> repack in
            ((x, delta), Pred (p, e1, e2))
        | k, 0 ->
            let d = delta / k in
            let e1 = Idtab.map (fun c -> c * d) tab1
                     |> Idtab.add x 1 |> repack in
            let e2 = Idtab.map (fun c -> c * d) tab2 |> repack in
            ((x, delta), Pred (p, e1, e2))
        | _ -> assert false
      end
    | _ -> super#boolex (x, delta) e
end

let assert_qf =
  let module M = struct exception Not end in
  let test = object (self)
    inherit [unit] traverse as super

    method! boolex () e =
      match e with
      | Forall _ | Exists _ -> raise M.Not
      | _ -> super#boolex () e
  end in
  fun e ->
    try test#boolex () e with
    | M.Not ->
        Printf.sprintf "assert_qf: %s not quantifier free" (boolex_to_string e)
        |> failwith

let reset_coefficient = object (self)
    inherit [ident] map as super

    method! boolex x e =
      assert_qf e ;
      let _, e = collect_terms#boolex x e in
      let (_, delta) = get_lcm#boolex (x, 1) e in
      let _, e = set_lcm#boolex (x, delta) e in
      (x, And (e, Pred (Div, Const delta, Var x)))
end

(******************************************************************************)

let no_smallest_map = object (self)
  inherit [ident * int] map as super

  method! boolex (x, l) e =
    match e with
    | Pred ((Div | Ndiv), Const k, _) ->
        ((x, lcm k l), e)
    | Pred (Lt, e1, e2) -> begin
        let (k1, _) = unpack_for x e1 in
        let (k2, _) = unpack_for x e2 in
        match k1, k2 with
        | 0, 0 -> ((x, l), e)
        | 0, k -> ((x, l), False)
        | k, 0 -> ((x, l), True)
        | _ -> assert false
      end
    | _ -> super#boolex (x, l) e
end

let subst = object (self)
  inherit [ident * intex] map as super

  method! intex (x, t) e =
    match e with
    | Var y when x = y -> ((x, t), t)
    | _ -> super#intex (x, t) e
end

let init fn low high =
  let rec spin cur =
    if cur > high then [] else
    fn cur :: spin (cur + 1)
  in
  spin low

let disjoin = function
  | [] -> False
  | e :: es -> List.fold_left (fun e1 e2 -> Or (e1, e2)) e es

let no_smallest x e =
  let (_, f) = reset_coefficient#boolex x e in
  let ((_, delta), f) = no_smallest_map#boolex (x, 1) f in
  init (fun i -> subst#boolex (x, Const i) f |> snd) 1 delta |>
  disjoin |> simplify

let smallest_map = object (self)
  inherit [ident * int * intex list] traverse as super

  method! boolex (x, l, bset) e =
    match e with
    | Pred ((Div | Ndiv), Const k, _) ->
        (x, lcm k l, bset)
    | Pred (Lt, e1, e2) -> begin
        let (k1, _) = unpack_for x e1 in
        let (k2, _) = unpack_for x e2 in
        let bset = if k1 <> 0 && k2 = 0 then e2 :: bset else bset in
        (x, l, bset)
      end
    | _ -> super#boolex (x, l, bset) e
end

let smallest x e =
  let (_, f) = reset_coefficient#boolex x e in
  let (_, delta, bset) = smallest_map#boolex (x, 1, []) f in
  let g = List.map begin fun b ->
      subst#boolex (x, Add (Var x, b)) f |> snd
    end bset |> disjoin in
  init (fun i -> subst#boolex (x, Const i) g |> snd) 1 delta |>
  disjoin |> simplify

(*****************************************************************************)

let eliminate_quantifiers = object (self)
  inherit [unit] map as super

  method! boolex () e =
    match e with
    | Exists (x, e) ->
        let _, e = self#boolex () e in
        let f1 = no_smallest x e in
        let f2 = smallest x e in
        ((), Or (f1, f2) |> simplify)
    | Forall (x, e) ->
        let _, e = negation_normalize#boolex false e in
        let _, e = self#boolex () (Exists (x, e)) in
        let _, e = negation_normalize#boolex false e in
        ((), e)
    | _ -> super#boolex () e
end

let decide e =
  let (), f = eliminate_quantifiers#boolex () (simplify e) in
  match f with
  | True -> true
  | False -> false
  | _ ->
      Printf.printf "Cannot decide %s\nResidual problem: %s\n"
        (boolex_to_string e)
        (boolex_to_string f) ;
      failwith "decide"
