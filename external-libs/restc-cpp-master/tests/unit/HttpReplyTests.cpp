
// Include before boost::log headers
#include "restc-cpp/logging.h"

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

#include "../src/ReplyImpl.h"

#include "restc-cpp/test_helper.h"
#include "lest/lest.hpp"


using namespace std;
using namespace restc_cpp;

namespace restc_cpp{
namespace unittests {

using test_buffers_t = std::list<std::string>;

class MockReader : public DataReader {
public:
    MockReader(test_buffers_t& buffers)
    : test_buffers_{buffers} {

        next_buffer_ = test_buffers_.begin();
    }

    bool IsEof() const override {
        return next_buffer_ == test_buffers_.end();
    }

    boost::asio::const_buffers_1 ReadSome() override {
        if (IsEof()) {
            return {nullptr, 0};
        }

        size_t data_len = next_buffer_->size();
        const char * const data = next_buffer_->c_str();
        ++next_buffer_;
        return {data, data_len};
    }

    test_buffers_t& test_buffers_;
    test_buffers_t::iterator next_buffer_;
};

class TestReply : public ReplyImpl
{
public:
    TestReply(Context& ctx, RestClient& owner, test_buffers_t& buffers)
    : ReplyImpl(nullptr, ctx, owner), buffers_{buffers}
    {
    }

    void SimulateServerReply() {
        StartReceiveFromServer(make_unique<MockReader>(buffers_));
    }

private:
    test_buffers_t& buffers_;
};


} // unittests
} // restc_cpp


const lest::test specification[] = {
STARTCASE(TestSimpleHeader)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n"
        "Server: Cowboy\r\n"
        "Connection: keep-alive\r\n"
        "X-Powered-By: Express\r\n"
        "Vary: Origin, Accept-Encoding\r\n"
        "Cache-Control: no-cache\r\n"
        "Pragma: no-cache\r\n"
        "Expires: -1\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Content-Length: 0\r\n"
        "Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n"
        "\r\n");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();

         CHECK_EQUAL("Cowboy", *reply.GetHeader("Server"));
         CHECK_EQUAL("keep-alive", *reply.GetHeader("Connection"));
         CHECK_EQUAL("Express", *reply.GetHeader("X-Powered-By"));
         CHECK_EQUAL("Origin, Accept-Encoding", *reply.GetHeader("Vary"));
         CHECK_EQUAL("no-cache", *reply.GetHeader("Cache-Control"));
         CHECK_EQUAL("no-cache", *reply.GetHeader("Pragma"));
         CHECK_EQUAL("-1", *reply.GetHeader("Expires"));
         CHECK_EQUAL("application/json; charset=utf-8", *reply.GetHeader("Content-Type"));
         CHECK_EQUAL("Thu, 21 Apr 2016 13:44:36 GMT", *reply.GetHeader("Date"));
         CHECK_EQUAL("0", *reply.GetHeader("Content-Length"));

     }).get();
} ENDCASE

STARTCASE(TestSimpleSegmentedHeader)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n");
    buffer.push_back("Server: Cowboy\r\n");
    buffer.push_back("Connection: keep-alive\r\n");
    buffer.push_back("X-Powered-By: Express\r\n");
    buffer.push_back("Vary: Origin, Accept-Encoding\r\n");
    buffer.push_back("Cache-Control: no-cache\r\n");
    buffer.push_back("Pragma: no-cache\r\n");
    buffer.push_back("Expires: -1\r\n");
    buffer.push_back("Content-Type: application/json; charset=utf-8\r\n");
    buffer.push_back("Content-Length: 0\r\n");
    buffer.push_back("Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n");
    buffer.push_back("\r\n");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();

         CHECK_EQUAL("keep-alive", *reply.GetHeader("Connection"));
         CHECK_EQUAL("0", *reply.GetHeader("Content-Length"));

     }).get();
} ENDCASE

STARTCASE(TestSimpleVerySegmentedHeader)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\nSer");
    buffer.push_back("ver: Cowboy\r\n");
    buffer.push_back("Connection: keep-alive\r");
    buffer.push_back("\nX-Powered-By: Express\r\nV");
    buffer.push_back("ary");
    buffer.push_back(": Origin, Accept-Encoding\r\nCache-Control: no-cache\r\n");
    buffer.push_back("Pragma: no-cache\r\n");
    buffer.push_back("Expires: -1\r\n");
    buffer.push_back("Content-Type: application/json; charset=utf-8\r\n");
    buffer.push_back("Content-Length: 0\r\n");
    buffer.push_back("Date: Thu, 21 Apr 2016 13:44:36 GMT");
    buffer.push_back("\r");
    buffer.push_back("\n");
    buffer.push_back("\r");
    buffer.push_back("\n");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();

         CHECK_EQUAL("keep-alive", *reply.GetHeader("Connection"));
         CHECK_EQUAL("0", *reply.GetHeader("Content-Length"));

     }).get();
} ENDCASE

STARTCASE(TestSimpleBody)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n"
        "Server: Cowboy\r\n"
        "Connection: keep-alive\r\n"
        "Vary: Origin, Accept-Encoding\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Content-Length: 10\r\n"
        "Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n"
        "\r\n"
        "1234567890");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();
         auto body = reply.GetBodyAsString();

         CHECK_EQUAL("Cowboy", *reply.GetHeader("Server"));
         CHECK_EQUAL("10", *reply.GetHeader("Content-Length"));
         CHECK_EQUAL(10, (int)body.size());

     }).get();
} ENDCASE

STARTCASE(TestSimpleBody2)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n"
        "Server: Cowboy\r\n"
        "Connection: keep-alive\r\n"
        "Vary: Origin, Accept-Encoding\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Content-Length: 10\r\n"
        "Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n"
        "\r\n");
    buffer.push_back("1234567890");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();
         auto body = reply.GetBodyAsString();

         CHECK_EQUAL("Cowboy", *reply.GetHeader("Server"));
         CHECK_EQUAL("10", *reply.GetHeader("Content-Length"));
         CHECK_EQUAL(10, (int)body.size());

     }).get();
} ENDCASE

STARTCASE(TestSimpleBody3)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n"
        "Server: Cowboy\r\n"
        "Connection: keep-alive\r\n"
        "Vary: Origin, Accept-Encoding\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Content-Length: 10\r\n"
        "Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n"
        "\r\n");
    buffer.push_back("1234567");
    buffer.push_back("890");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();
         auto body = reply.GetBodyAsString();

         CHECK_EQUAL("Cowboy", *reply.GetHeader("Server"));
         CHECK_EQUAL("10", *reply.GetHeader("Content-Length"));
         CHECK_EQUAL(10, (int)body.size());

     }).get();
} ENDCASE

STARTCASE(TestSimpleBody4)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n"
        "Server: Cowboy\r\n"
        "Connection: keep-alive\r\n"
        "Vary: Origin, Accept-Encoding\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Content-Length: 10\r\n"
        "Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n"
        "\r\n12");
    buffer.push_back("34567");
    buffer.push_back("890");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();
         auto body = reply.GetBodyAsString();

         CHECK_EQUAL("Cowboy", *reply.GetHeader("Server"));
         CHECK_EQUAL("10", *reply.GetHeader("Content-Length"));
         CHECK_EQUAL(10, (int)body.size());

     }).get();
} ENDCASE

STARTCASE(TestChunkedBody)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n"
        "Server: Cowboy\r\n"
        "Connection: keep-alive\r\n"
        "Vary: Origin, Accept-Encoding\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Transfer-Encoding: chunked\r\n"
        "Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n"
        "\r\n"
        "4\r\nWiki\r\n5\r\npedia\r\nE\r\n in\r\n\r\nchunks."
        "\r\n0\r\n\r\n");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();
         auto body = reply.GetBodyAsString();

         CHECK_EQUAL("Cowboy", *reply.GetHeader("Server"));
         CHECK_EQUAL("chunked", *reply.GetHeader("Transfer-Encoding"));
         CHECK_EQUAL((0x4 + 0x5 + 0xE), (int)body.size());

     }).get();
} ENDCASE

STARTCASE(TestChunkedBody2)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n"
        "Server: Cowboy\r\n"
        "Connection: keep-alive\r\n"
        "Vary: Origin, Accept-Encoding\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Transfer-Encoding: chunked\r\n"
        "Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n"
        "\r\n");
    buffer.push_back("4\r\nWiki\r\n");
    buffer.push_back("5\r\npedia\r\n");
    buffer.push_back("E\r\n in\r\n\r\nchunks.\r\n");
    buffer.push_back("0\r\n\r\n");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();
         auto body = reply.GetBodyAsString();

         CHECK_EQUAL("Cowboy", *reply.GetHeader("Server"));
         CHECK_EQUAL("chunked", *reply.GetHeader("Transfer-Encoding"));
         CHECK_EQUAL((0x4 + 0x5 + 0xE), (int)body.size());

     }).get();
} ENDCASE

STARTCASE(TestChunkedBody4)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n"
        "Server: Cowboy\r\n"
        "Connection: keep-alive\r\n"
        "Vary: Origin, Accept-Encoding\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Transfer-Encoding: chunked\r\n"
        "Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n"
        "\r\n");
    buffer.push_back("4\r\nW");
    buffer.push_back("iki\r\n5\r\npedi");
    buffer.push_back("a\r\nE\r\n in\r\n\r\nchunks.\r");
    buffer.push_back("\n");
    buffer.push_back("0");
    buffer.push_back("\r");
    buffer.push_back("\n");
    buffer.push_back("\r");
    buffer.push_back("\n");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();
         auto body = reply.GetBodyAsString();

         CHECK_EQUAL("Cowboy", *reply.GetHeader("Server"));
         CHECK_EQUAL("chunked", *reply.GetHeader("Transfer-Encoding"));
         CHECK_EQUAL((0x4 + 0x5 + 0xE), (int)body.size());

     }).get();
} ENDCASE



STARTCASE(TestChunkedTrailer)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n"
        "Server: Cowboy\r\n"
        "Connection: keep-alive\r\n"
        "Vary: Origin, Accept-Encoding\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Transfer-Encoding: chunked\r\n"
        "Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n"
        "\r\n");
    buffer.push_back("4\r\nWiki\r\n");
    buffer.push_back("5\r\npedia\r\n");
    buffer.push_back("E\r\n in\r\n\r\nchunks.\r\n");
    buffer.push_back("0\r\n");
    buffer.push_back("Server: Indian\r\n");
    buffer.push_back("Connection: close\r\n");
    buffer.push_back("\r\n");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();

         CHECK_EQUAL("Cowboy", *reply.GetHeader("Server"));
         CHECK_EQUAL("keep-alive", *reply.GetHeader("Connection"));
         CHECK_EQUAL("chunked", *reply.GetHeader("Transfer-Encoding"));

         auto body = reply.GetBodyAsString();

         CHECK_EQUAL("Indian", *reply.GetHeader("Server"));
         CHECK_EQUAL("close", *reply.GetHeader("Connection"));
         CHECK_EQUAL("chunked", *reply.GetHeader("Transfer-Encoding"));
         CHECK_EQUAL((0x4 + 0x5 + 0xE), (int)body.size());

     }).get();
} ENDCASE

STARTCASE(TestChunkedParameterAndTrailer)
{
    ::restc_cpp::unittests::test_buffers_t buffer;

    buffer.push_back("HTTP/1.1 200 OK\r\n"
        "Server: Cowboy\r\n"
        "Connection: keep-alive\r\n"
        "Vary: Origin, Accept-Encoding\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Transfer-Encoding: chunked\r\n"
        "Date: Thu, 21 Apr 2016 13:44:36 GMT\r\n"
        "\r\n");
    buffer.push_back("4;test=1;tset=\"yyyy\"\r\nWiki\r\n");
    buffer.push_back("5;more-to-follow\r\npedia\r\n");
    buffer.push_back("E;77\r\n in\r\n\r\nchunks.\r\n");
    buffer.push_back("0;this-is-the-end\r\n");
    buffer.push_back("Server: Indian\r\n");
    buffer.push_back("Connection: close\r\n");
    buffer.push_back("\r\n");

     auto rest_client = RestClient::Create();
     rest_client->ProcessWithPromise([&](Context& ctx) {

         ::restc_cpp::unittests::TestReply reply(ctx, *rest_client, buffer);

         reply.SimulateServerReply();

         CHECK_EQUAL("Cowboy", *reply.GetHeader("Server"));
         CHECK_EQUAL("keep-alive", *reply.GetHeader("Connection"));
         CHECK_EQUAL("chunked", *reply.GetHeader("Transfer-Encoding"));

         auto body = reply.GetBodyAsString();

         CHECK_EQUAL("Indian", *reply.GetHeader("Server"));
         CHECK_EQUAL("close", *reply.GetHeader("Connection"));
         CHECK_EQUAL("chunked", *reply.GetHeader("Transfer-Encoding"));
         CHECK_EQUAL((0x4 + 0x5 + 0xE), (int)body.size());

     }).get();
} ENDCASE
}; //lest

int main( int argc, char * argv[] )
{
    namespace logging = boost::log;

    logging::core::get()->set_filter
    (
#ifdef _DEBUG
        logging::trivial::severity >= logging::trivial::trace
#else
        logging::trivial::severity >= logging::trivial::fatal
#endif
    );
    return lest::run( specification, argc, argv );
}
