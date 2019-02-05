# Introduction to the restc-cpp C++ library
<i>The magic that takes the pain out of accessing JSON API's from C++ </i>

<b>What it does:</b>
- It formulates a HTTP request to a REST API server. Then, it transforms
  the JSON formatted payload in the reply into a native C++ object (GET).
- It Serialize a native C++ object or a container of C++ objects into a JSON payload
  and send it to the REST API server (POST, PUT).
- It formulates a HTTP request to the REST API without serializing any data in either
  direction (typically DELETE).
- It uploads a stream of data, like a file, to a HTTP server.
- It downloads a stream of data, like a file or an array of JSON objects, from a HTTP server.

That's basically it. It does not solve world hunger.
It make no attempts to be a C++ framework.

You can use it's single components, like the powerful C++ HTTP Client to
send and receive non-JSON data as a native C++ replacement for libcurl.
You can use the template code that transforms data between C++ and JSON
for other purposes (for example in a REST API SERVER) - but the library
is designed and implemented for the single purpose of using C++ to
interact efficiently and effortless with external REST API servers.

The library is written by Jarle (jgaa) Aase, a senior freelance C++ developer
with roughly 30 years of experience in software development.

# Design Goals
The design goal of this project is to make external REST API's
simple and safe to use in C++ projects, but still fast and memory efficient.

Another goal was to use coroutines for the application logic that sends data to or
pulls data from the REST API servers. This makes the code easy to write
and understand, and also simplifies debugging and investigation of core dumps.
In short; the code executes asynchronously, but there are no visible callbacks
or completion functions. It looks like crystal clear,
old fashion, single threaded sequential code (using modern C++ language).
You don't sacrifice code clearness to achieve massive parallelism and
high performance. Coroutines was a strong motivation to write a new
C++ HTTP Client from scratch. To see how this actually works, please see the
 [modern async cpp example](https://github.com/jgaa/modern_async_cpp_example)).


Finally, in a world where the Internet is getting increasingly
[dangerous](http://www.dailydot.com/layer8/bruce-schneier-internet-of-things/),
and all kind of malicious parties, from your own government to international Mafia
(with Putin in Moscow and the Clown in the White House, the differences is
blurring out), search for vulnerabilities in your software stack to snoop, ddos,
intercept and blackmail you and your customers/users - I have a strong emphasis
on security in all software projects I'm involved in. I have limited the
dependencies on third party libraries as much as I could (I still use OpenSSL
which is a snakes nest of of yet undisclosed vulnerabilities - but as of now
there are no alternatives that works out of the box with boost::asio).
I have also tried to imagine any possible way a malicious API server
could try to attack you (by exploiting or exceeding local resources - like sending
a malicious compressed package that expands to a petabyte of zeros) and designed
to detect any potential problems and break out of it by throwing an exception as
soon as possible.

# Why?
In the spring of 2016 I was asked to implement a SDK for a REST API in
several languages. For Python, Java and Ruby it was trivial to make a simple
object oriented implementation. When I started planning the C++ implementation of the
SDK, I found no suitable, free libraries. I could not even find a proper HTTP Client
implementation(!). I could have solved the problem using QT - but i found it
overkill to use a huge GUI framework for C++ code that are most likely to run
in high performance servers - and that may end up in projects using some other
C++ framework that can't coexist with QT.

Many years ago I designed and implemented a C++ REST Client for an early
version of Amazon AWS using libcurl - and - well, I had no strong urge to repeat
that experience. So I spent a few weeks creating my own HTTP Client library
using boost::asio with JSON serialization/deserialization.

# Dependencies
Restc-cpp depends on C++14 with its standard libraries and:
  - boost
  - rapidjson (mature, ultra-fast, json sax, header-only library)
  - lest (Unit test header only library) (If compiled with testing enabled)
  - openssl or libressl (If compiled with TLS support)
  - zlib (If compiled with compression support)

rapidjson and lest are included as CMake external dependencies.

# License
MIT license. It is Free. Free as in Free Beer. Free as in Free Air.

# Examples

## Fetch raw data
The following code demonstrates how to run a simple HTTP request asynchronously,
using the co-routine support in boost::asio behind the scenes.


```C++
#include <iostream>
#include "restc-cpp/restc-cpp.h"

using namespace std;
using namespace restc_cpp;

void DoSomethingInteresting(Context& ctx) {
    // Here we are in a co-routine, running in a worker-thread.

    // Asynchronously connect to a server and fetch some data.
    auto reply = ctx.Get("http://jsonplaceholder.typicode.com/posts/1");

    // Asynchronously fetch the entire data-set and return it as a string.
    auto json = reply->GetBodyAsString();

    // Just dump the data.
    cout << "Received data: " << json << endl;
}

int main() {
    auto rest_client = RestClient::Create();

    // Call DoSomethingInteresting as a co-routine in a worker-thread.
    rest_client->Process(DoSomethingInteresting);

    // Wait for the coroutine to finish, then close the client.
    rest_client->CloseWhenReady(true);
}
```

And here is the output you could expect
```text
Received data: {
  "userId": 1,
  "id": 1,
  "title": "sunt aut facere repellat provident occaecati excepturi optio reprehenderit",
  "body": "quia et suscipit\nsuscipit recusandae consequuntur expedita et cum\nreprehenderit molestiae ut ut quas totam\nnostrum rerum est autem sunt rem eveniet architecto"
}
```


## Fetch a C++ object from a server that serialize to JSON

Here is a sightly more interesting example, using JSON
serialization, and some modern C++ features.

```C++
#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/fusion/adapted.hpp>

#include "restc-cpp/restc-cpp.h"
#include "restc-cpp/RequestBuilder.h"

using namespace std;
using namespace restc_cpp;

// C++ structure that match the JSON entries received
// from http://jsonplaceholder.typicode.com/posts/{id}
struct Post {
    int userId = 0;
    int id = 0;
    string title;
    string body;
};

// Since C++ does not (yet) offer reflection, we need to tell the library how
// to map json members to a type. We are doing this by declaring the
// structs/classes with BOOST_FUSION_ADAPT_STRUCT from the boost libraries.
// This allows us to convert the C++ classes to and from JSON.

BOOST_FUSION_ADAPT_STRUCT(
    Post,
    (int, userId)
    (int, id)
    (string, title)
    (string, body)
)

// The C++ main function - the place where any adventure starts
int main() {
    // Create an instance of the rest client
    auto rest_client = RestClient::Create();

    // Create and instantiate a Post from data received from the server.
    Post my_post = rest_client->ProcessWithPromiseT<Post>([&](Context& ctx) {
        // This is a co-routine, running in a worker-thread

        // Instantiate a Post structure.
        Post post;

        // Serialize it asynchronously. The asynchronously part does not really matter
        // here, but it may if you receive huge data structures.
        SerializeFromJson(post,

            // Construct a request to the server
            RequestBuilder(ctx)
                .Get("http://jsonplaceholder.typicode.com/posts/1")

                // Add some headers for good taste
                .Header("X-Client", "RESTC_CPP")
                .Header("X-Client-Purpose", "Testing")

                // Send the request
                .Execute());

        // Return the post instance trough a C++ future<>
        return post;
    })

    // Get the Post instance from the future<>, or any C++ exception thrown
    // within the lambda.
    .get();

    // Print the result for everyone to see.
    cout << "Received post# " << my_post.id << ", title: " << my_post.title;
}
```

The code above should return something like:
```text
Received post# 1, title: sunt aut facere repellat provident occaecati excepturi optio reprehenderit
```

Please refer to the [tutorial](doc/Tutorial.md) for more examples.

# Features
- High level Request Builder interface (similar to Java HTTP Clients) for convenience.
- Low level interface to create requests.
- All network IO operations are asynchronous trough boost::asio.
  - Use your own asio io-services
  - Let the library create and deal with the asio io-services
  - Use your own worker threads
  - Let the library create and deal with worker-threads
- Uses C++ / boost coroutines for application logic.
- HTTP Redirects.
- HTTP Basic Authentication.
- Logging trough boost::log or trough your own log macros.
- Connection Pool for fast re-use of existing server connections.
- Compression (gzip, deflate).
- JSON serialization to and from native C++ objects.
  - Optional Mapping between C++ property names and JSON 'on the wire' names.
  - Option to tag property names as read-only to filter them out when the C++ object is serialized for transfer to the server.
  - Filters out empty C++ properties when the C++ object is serialized for transfer to the server (can be disabled).
  - Iterator interface to received JSON lists of objects.
  - Memory constraint on incoming objects (to limit damages from rouge or buggy REST servers).
  - Serialization directly from std::istream to C++ object.
- Plain or chunked outgoing HTTP payloads.
- Several strategies for lazy data fetching in outgoing requests.
  - Override RequestBody to let the library pull for data when required.
  - Write directly to the outgoing DataWriter when data is required.
  - Just provide a C++ object and let the library serialize it directly to the wire.

# Current Status
The project is maturing fast. There are no known bugs.

The project has been in public BETA since April 11th 2017.


# Supported operating systems
These are the operating systems I test with before releasing a new version.

 - Debian Stable (Jessie)
 - Debian Testing
 - Windows 10 / Microsoft Visual Studio 2015 and 2017, Community version / Professional version (All tests now passes with the latest release version of Docker for Windows)
 - Fedora 25
 - Ubuntu Server LTS
 - macOS Sierra (OS X)


# Similar projects
  - [JSON for Modern C++](https://nlohmann.github.io/json/) by Niels Lohmann.
  - [json11 - tiny JSON library for C++11, providing JSON parsing and serialization](https://github.com/dropbox/json11)


# More information
- [Getting started](doc/GettingStarted.md)
- [Tutorial](doc/Tutorial.md)
- [Build for Linux](https://github.com/jgaa/restc-cpp/wiki/Building-under-Linux)
- [Build for macOS](https://github.com/jgaa/restc-cpp/wiki/Building-under-macOS)
- [Build for Windows](https://github.com/jgaa/restc-cpp/wiki/Building-under-Windows)
- [Running the tests](doc/RunningTheTests.md)
- [Planned work](https://github.com/jgaa/restc-cpp/wiki)
- [How to link your program with restc-cpp from the command-line](https://github.com/jgaa/restc-cpp/tree/master/examples/cmdline)
- [How to use cmake to find and link with restc-cpp from your project](https://github.com/jgaa/restc-cpp/tree/master/examples/cmake_normal)
- [How to use restc-cpp as an external cmake module from your project](https://github.com/jgaa/restc-cpp/tree/master/examples/cmake_external_project)
