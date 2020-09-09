for file in ./inputs/*
do
  echo "Executing $file..."; cp "$file" input.txt; gtimeout 60s time ./project1cs360s2020; cat output.txt; rm output.txt
done
