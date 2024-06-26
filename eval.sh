find . -type f -name '*.csv' -exec sh -c 'evo_traj euroc "$0" --save_as_tum' {} \;

for file in *.tum; do if [ "$file" != "data.tum" ]; then evo_rpe tum data.tum "$file"  --align --pose_relation angle_deg --save_results "${file%.tum}r.zip"; fi; done
for file in *.tum; do if [ "$file" != "data.tum" ]; then evo_rpe tum data.tum "$file"  --align --pose_relation trans_part --save_results "${file%.tum}t.zip"; fi; done
for file in *.tum; do if [ "$file" != "data.tum" ]; then evo_rpe tum data.tum "$file"  --align --pose_relation full --save_results "${file%.tum}.zip"; fi; done
for file in *.tum; do if [ "$file" != "data.tum" ]; then evo_ape tum data.tum "$file"  --align --save_results "${file%.tum}APE.zip"; fi; done

evo_res m.zip c.zip i.zip s.zip --use_filenames --save_table ra.csv
evo_res mt.zip ct.zip it.zip st.zip --use_filenames --save_table T.csv
evo_res mr.zip cr.zip ir.zip sr.zip --use_filenames --save_table R.csv
evo_res mAPE.zip cAPE.zip iAPE.zip sAPE.zip --use_filenames --save_table APE.csv

rm IP*
evo_res *.tum.zip -p
evo_res *.tumr.zip -p
evo_res *.tumt.zip -p
evo_res *APE.zip -p
evo_traj tum AOPnP.tum MLPnP.tum SQPnP.tum --align --ref=GT.tum -p