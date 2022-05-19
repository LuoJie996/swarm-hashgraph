for ((i=0;i<6;i++))
do
docker stop robot$i
done

for ((i=0;i<6;i++))
do
docker rm robot$i
done

for ((i=0;i<6;i++))
do
gnome-terminal -e 'bash -c "docker run -ti -u root --name robot'$i' --hostname robot'$i'  robot0 /bin/bash; exec bash"'
done
