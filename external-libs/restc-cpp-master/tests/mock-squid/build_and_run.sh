#!/bin/sh

docker stop restc-cpp-squid
docker rm restc-cpp-squid
docker build . -t jgaa/restc-cpp-mock-squid
docker run --name restc-cpp-squid --link restc-cpp-nginx:api.example.com -d -p 3003:3128  -t jgaa/restc-cpp-mock-squid

# There is a problem during boot where squid is unable to
# resolve the linked hostname
sleep 2

case "$OSTYPE" in
  msys*)
	echo "WINDOWS"
	winpty docker exec -it restc-cpp-squid service squid3 restart
	;;
  *)
	docker exec -it restc-cpp-squid service squid3 restart
	;;
esac

