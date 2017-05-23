let n = ref 0

let f th =
  for i = 1 to 10000 do
    let x = !n in
    Printf.printf "%d: %d\n%!" th x;
    n := x + 1;
  done
      