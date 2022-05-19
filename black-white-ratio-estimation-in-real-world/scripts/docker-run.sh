for ((i=0;i<6;i++))
do
    docker run -ti -u root --name robot$i --hostname robot$i robot0 "/bin/bash"
done


