rm *.zip

mv CPnP.tum AOPnP.tum;

./eval.sh

evo_res *APE.zip -p

evo_traj tum AOPnP.tum IPnP.tum MLPnP.tum SQPnP.tum --align --ref=data.tum -p
