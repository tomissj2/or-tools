FROM alpine as build
 
RUN apk add alpine-sdk linux-headers cmake lsb-release-minimal
 
COPY . /src
WORKDIR /src
 
RUN cmake -S . -B build -DBUILD_DEPS=ON
RUN cmake --build build --config Release --target vrp_capacity -j 10 -v
 
 
FROM alpine
 
RUN apk add libstdc++
 
WORKDIR /or-tools
 
COPY --from=build /src/build/bin/vrp_capacity ./bin/
COPY --from=build /src/build/lib/libortools.so ./lib/
COPY --from=build /src/build/lib/libortools.so.9 ./lib/

ENTRYPOINT ["tail", "-f", "/home"]

EXPOSE 8080