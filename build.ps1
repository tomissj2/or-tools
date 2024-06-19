$imageName = "cpp-routing-build-image"
$containerName = "cppRouringBuildContainer"
$portMapping = "8080:8080"

rm -r build_result

mkdir build_result
mkdir build_result/bin
mkdir build_result/lib

docker build -t $imageName .
docker run -d -p $portMapping --name $containerName $imageName

docker cp cppRouringBuildContainer:/or-tools/bin/vrp_capacity ./build_result/bin/
docker cp cppRouringBuildContainer:/or-tools/lib/libortools.so ./build_result/lib/
docker cp cppRouringBuildContainer:/or-tools/lib/libortools.so.9 ./build_result/lib/

docker rm -f $containerName

docker rmi -f $imageName