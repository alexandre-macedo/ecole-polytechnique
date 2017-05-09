class type node = object
  (** is it (left-) associative? *)
  method is_assoc  : bool
  (** what is its priority? *)
  method priority  : int
  (** what is its stringified form? *)
  method to_string : string
end

val exact   : string -> node
val unop    : int -> string -> node -> node
val binop   : int -> bool -> string -> node -> node -> node

module Test = struct
  let x = exact "x"
  let y = exact "y"
  let z = exact "z"
  let xpy = binop 1 true " + " x y
  let () = assert (xpy#to_string = "x + y")
  let xpytz = binop 2 true " * " xpy z
  let () = assert (xpytz#to_string = "(x + y) * z")
  let xty = binop 2 true " * " x y
  let () = assert (xty#to_string = "x * y")
  let xtypz = binop 1 true " + " xty z
  let () = assert (xtypz#to_string = "x * y + z")
  let xtytz = binop 2 true " * " xty z
  let () = assert (xtytz#to_string = "x * y * z")
  let xmynz = binop 2 false " m " x (binop 3 false " n " y z)
  let () = assert (xmynz#to_string = "x m y n z")
  let xnymz = binop 3 false " n " x (binop 2 false " m " y z)
  let () = assert (xnymz#to_string = "x n (y m z)")
end