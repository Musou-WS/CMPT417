# CMPT417
## Run Individual Instances:
"python run_experiments.py --instance "instances/test_*" --solver CBS --batch" (CBS) (resultsCBS.csv)

"python run_experiments.py --instance "instances/test_*" --solver CBSWH --batch" (DG) (resultsCBSWH.csv)

"python run_experiments.py --instance "instances/test_*" --solver CBSWDGH --batch" (WDG) (resultsCBSWDGH.csv)

"python run_experiments.py --instance "instances/test_*" --solver CBSWHL --batch" (Lazy A*) (resultsCBSWHL.csv)

## Run Nathan's Instances:
"python run_test.py --instance "NathanInstances/random-32-32-20.map" --agents "NathanInstances/scen-random/random-32-32-20-random-*.scen" --num 10 --solver CBSWH --batch" (DG) (resultsCBSWH.csv)

"python run_test.py --instance "NathanInstances/random-32-32-20.map" --agents "NathanInstances/scen-random/random-32-32-20-random-*.scen" --num 10 --solver CBSWDGH --batch" (WDG) (resultsCBSWDGH.csv)

"python run_test.py --instance "NathanInstances/random-32-32-20.map" --agents "NathanInstances/scen-random/random-32-32-20-random-*.scen" --num 10 --solver CBSWHL --batch" (Lazy A*) (resultsCBSWHL.csv)