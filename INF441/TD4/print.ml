module type Printer = sig
  type t
  val print : t -> string
end

module IntPrinter : Printer with type t = int = struct
  type t = int
  let print = string_of_int
end

module BoolPrinter : Printer with type t = bool = struct
  type t = bool
  let print = string_of_bool
end

module PairPrinter (P1 : Printer) (P2 : Printer) :
  Printer with type t = P1.t * P2.t =
struct
  type t = P1.t * P2.t

  let print (a, b) =
    String.concat "" ["(" ; P1.print a ; "," ; P2.print b ; ")"]
end

module ListPrinter (P : Printer) :
  Printer with type t = P.t list =
struct
  type t = P.t list

  let rec print_aux rest = function
    | [] -> List.rev ("]" :: rest)
    | [x] -> List.rev ("]" :: P.print x :: rest)
    | x :: xs ->
        print_aux ("," :: P.print x :: rest) xs

  let print xs = String.concat "" (print_aux ["["] xs)
end

let p_listpairintbool : (int * bool) list -> string =
  let module P = ListPrinter (PairPrinter (IntPrinter) (BoolPrinter)) in
  P.print
