find . -type f -name '*.csv' -exec sh -c 'evo_traj euroc "$0" --save_as_tum' {} \;

for file in *.tum; do if [ "$file" != "data.tum" ]; then evo_rpe tum "$file" data.tum --align --pose_relation full --save_results "${file%.tum}_all.zip"; fi; done


for n in 2000; do 
    evo_res m2k.zip c2k.zip i2k.zip s2k.zip --use_filenames --save_table all_$n.csv
done