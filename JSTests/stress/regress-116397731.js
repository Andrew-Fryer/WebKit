var out;

function func0(value) {
  if (value) {}
  out = value;
  value = value + 1 + 2;
}

function main() {
  for (let i = 0; i < testLoopCount; i++)
    func0(undefined);
  if (out !== undefined)
    throw new Error(`Bad value: ${out}!`);
}

main();
