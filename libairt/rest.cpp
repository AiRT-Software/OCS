#include "rest.h"
#include <iostream>
#include <log.h>

// for sending http requests
#include <restc-cpp/restc-cpp.h>
#include <restc-cpp/RequestBuilder.h>

// for changing restc-cpp log level
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>


using airt::Rest;
using airt::Log;


Rest::Rest()
{
    // set the logging level of the http requests
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= boost::log::trivial::debug);
}

Rest::~Rest()
{
}

bool Rest::httpGet(std::string &reply_body, const std::string &endpoint)
{
    bool ok = false;
    std::string exception_what;

    // you will need to create the object locally
    rest_client = restc_cpp::RestClient::CreateUseOwnThread();

    auto cprop = rest_client->GetConnectionProperties();
//    cprop->connectTimeoutMs = cprop->connectTimeoutMs*3;
//    cprop->sendTimeoutMs = cprop->sendTimeoutMs*3;
    cprop->replyTimeoutMs = cprop->replyTimeoutMs*3;
    cprop->recvTimeout = cprop->recvTimeout*3;

    std::unique_ptr<restc_cpp::Reply> reply;

    rest_client->Process([&](restc_cpp::Context &ctx) {
        restc_cpp::RequestBuilder rb = restc_cpp::RequestBuilder(ctx);
        rb.Header("Accept", "application/json");
        rb.Get(endpoint);

        try
        {
            // Execute will throw an exception (connection refused) if the pozyxserver is not running
            reply = rb.Execute();
        }
        catch (const std::exception &ex)
        {
            exception_what = ex.what();
            rest_client->CloseWhenReady();
            return;
        }

        // Dump the data
        reply_body = reply->GetBodyAsString();

        ok = true;

        // Shut down the io-service. This will cause run() (below) to return.
        rest_client->CloseWhenReady();
    });

    // Start the io-service, using this thread.
    rest_client->GetIoService().run();

    if (ok)
        Log::info("httpGet: raw-Response: {}", reply_body);
    else
        Log::critical("httpGet: no response was obtained from {}. Error: {}", endpoint, exception_what);

    return ok;
}

bool Rest::httpPost(std::string &reply_body, const std::string &endpoint, const std::string & payload)
{
    bool ok = false;
    std::string exception_what;

    rest_client = restc_cpp::RestClient::CreateUseOwnThread();

    auto cprop = rest_client->GetConnectionProperties();
//    cprop->connectTimeoutMs = cprop->connectTimeoutMs*3;
//    cprop->sendTimeoutMs = cprop->sendTimeoutMs*3;
    cprop->replyTimeoutMs = cprop->replyTimeoutMs*3;
    cprop->recvTimeout = cprop->recvTimeout*3;

    std::unique_ptr<restc_cpp::Reply> reply;

    rest_client->Process([&](restc_cpp::Context &ctx) {
        restc_cpp::RequestBuilder rb = restc_cpp::RequestBuilder(ctx);
        rb.Post(endpoint);
        rb.Header("Accept", "application/json");
        rb.Header("Content-Type", "application/json");
        rb.Data(payload);

        try
        {
            reply = rb.Execute();
        }
        catch (const std::exception &ex)
        {
            exception_what = ex.what();
            rest_client->CloseWhenReady();
            return;
        }

        reply_body = reply->GetBodyAsString();
        ok = true;
        rest_client->CloseWhenReady();
    });

    rest_client->GetIoService().run();

    if (ok)
        Log::info("httpPost: raw-Response: {}", reply_body);
    else
        Log::critical("httpPost: no response was obtained from {}. Error: {}", endpoint, exception_what);

    return ok;
}
