class type node = object
  method is_assoc : bool
  method priority : int
  method to_string : string
end

let exact str : node = object
  method is_assoc = false
  method priority = -1
  method to_string = str
end

let bracket_if fn (nd : node) : string =
  if nd#priority >= 0 && fn nd
  then "(" ^ nd#to_string ^ ")"
  else nd#to_string

let unop opprio opstr nd : node =
  let bracket = bracket_if (fun nd -> nd#priority < opprio) in
  object
    method is_assoc = false
    method priority = opprio
    method to_string = opstr ^ bracket nd
  end

let binop opprio opass opstr nd1 nd2 : node =
  let bracket1 = bracket_if begin fun nd ->
      nd#priority < opprio ||
      (nd#priority = opprio && not (opass || nd#is_assoc))
    end in
  let bracket2 = bracket_if begin fun nd ->
      nd#priority < opprio
    end in
  object
    method is_assoc = opass
    method priority = opprio
    method to_string = bracket1 nd1 ^ opstr ^ bracket2 nd2
  end

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
  let t = binop 1 true " + " (binop 1 true " + " y (unop 2 "~ " x)) z
end
