(** identifiers *)
type ident = string

(** integer valued expressions *)
type intex =
  | Const of int                         (** constants *)
  | Var   of ident                       (** variables *)
  | Add   of intex * intex
  | Sub   of intex * intex
  | Times of intex * intex
  | Neg   of intex                       (** integer negation *)

(** relations between integer expressions *)
type rel = Eq | Lt | Leq | Gt | Geq | Neq

(** boolean valued expressions, aka assertions *)
type boolex =
  | Rel    of rel * intex * intex        (** relations *)
  | And    of boolex * boolex | True     (** conjunction and truth *)
  | Or     of boolex * boolex | False    (** disjunction and falsehood *)
  | Impl   of boolex * boolex            (** implication *)
  | Not    of boolex                     (** boolean negation *)
  | Forall of ident * boolex             (** universal *)
  | Exists of ident * boolex             (** existential *)

(** traverse class *)
  class ['acc] traverse = object (self)
  method intex (acc : 'acc) (e : intex) : 'acc =
    match e with
    | Const k -> acc
    | Add (e1, e2) ->
        let acc = self#intex acc e1 in
        let acc = self#intex acc e2 in
        acc
    (* ... *)

  method boolex (acc : 'acc) (e : boolex) : 'acc =
    (* ... *)
end