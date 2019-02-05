
#include <assert.h>

#include<boost/tokenizer.hpp>

#include "restc-cpp/logging.h"
#include "restc-cpp/helper.h"
#include "restc-cpp/error.h"
#include "restc-cpp/DataReaderStream.h"
#include "restc-cpp/url_encode.h"

#include "ReplyImpl.h"


using namespace std;

namespace restc_cpp {


boost::optional<string> ReplyImpl::GetHeader(const string& name) {
    boost::optional<string> rval;

    auto it = headers_.find(name);
    if (it != headers_.end()) {
        rval = it->second;
    }

    return rval;
}

ReplyImpl::ReplyImpl(Connection::ptr_t connection,
                     Context& ctx,
                     RestClient& owner,
                     Request::Properties::ptr_t& properties)
: connection_{move(connection)}, ctx_{ctx}
, properties_{properties}
, owner_{owner}
, connection_id_(connection_ ? connection_->GetId()
    : boost::uuids::random_generator()())
{
}

ReplyImpl::ReplyImpl(Connection::ptr_t connection,
                     Context& ctx,
                     RestClient& owner)
: connection_{move(connection)}, ctx_{ctx}
, properties_{owner.GetConnectionProperties()}
, owner_{owner}
, connection_id_(connection_ ? connection_->GetId()
    : boost::uuids::random_generator()())
{
}


ReplyImpl::~ReplyImpl() {
    if (connection_ && connection_->GetSocket().IsOpen()) {
        try {
            RESTC_CPP_LOG_TRACE << "~ReplyImpl(): " << *connection_
                << " is still open. Closing it to prevent problems with partially "
                << "received data.";
            connection_->GetSocket().Close();
            connection_.reset();
        } catch(std::exception& ex) {
            RESTC_CPP_LOG_WARN << "~ReplyImpl(): Caught exception:" << ex.what();
        }
    }
}

void ReplyImpl::StartReceiveFromServer(DataReader::ptr_t&& reader) {
    if (reader_) {
        throw RestcCppException("StartReceiveFromServer() is already called.");
    }

    static const auto timer_name = "StartReceiveFromServer"s;

    auto timer = IoTimer::Create(timer_name,
                                     properties_->replyTimeoutMs,
                                     connection_);

    assert(reader);
    auto stream = make_unique<DataReaderStream>(move(reader));
    stream->ReadServerResponse(response_);
    stream->ReadHeaderLines(
        [this](std::string&& name, std::string&& value) {
        headers_[move(name)] = move(value);
    });

    HandleContentType(move(stream));
    HandleConnectionLifetime();
    HandleDecompression();
    CheckIfWeAreDone();
}

void ReplyImpl::HandleContentType(unique_ptr<DataReaderStream>&& stream) {
    static const std::string content_len_name{"Content-Length"};
    static const std::string transfer_encoding_name{"Transfer-Encoding"};
    static const std::string chunked_name{"chunked"};

    if (const auto cl = GetHeader(content_len_name)) {
        content_length_ = stoi(*cl);
        reader_ = DataReader::CreatePlainReader(*content_length_, move(stream));
    } else {
        auto te = GetHeader(transfer_encoding_name);
        if (te && ciEqLibC()(*te, chunked_name)) {
            reader_ = DataReader::CreateChunkedReader([this](string&& name, string&& value) {
                headers_[move(name)] = move(value);
            },  move(stream));
        } else {
            reader_ = DataReader::CreateNoBodyReader();
        }
    }
}

void ReplyImpl::HandleConnectionLifetime() {
    static const std::string connection_name{"Connection"};
    static const std::string close_name{"close"};

    // Check for Connection: close header and tag the
    // connection for close
    const auto conn_hdr = GetHeader(connection_name);
    if (conn_hdr && ciEqLibC()(*conn_hdr, close_name)) {
        if (connection_) {
            RESTC_CPP_LOG_TRACE << "'Connection: close' header. "
                << "Tagging " << *connection_ << " for close.";
        }
        do_close_connection_ = true;
    }
}

void ReplyImpl::HandleDecompression() {
    static const std::string content_encoding{"Content-Encoding"};
    static const std::string gzip{"gzip"};
    static const std::string deflate{"deflate"};

    const auto te_hdr = GetHeader(content_encoding);
    if (!te_hdr) {
        return;
    }

    boost::tokenizer<> tok(*te_hdr);
    for(auto it = tok.begin(); it != tok.end(); ++it) {
#ifdef RESTC_CPP_WITH_ZLIB
        if (ciEqLibC()(gzip, *it)) {
            RESTC_CPP_LOG_TRACE << "Adding gzip reader to " << *connection_;
            reader_ = DataReader::CreateGzipReader(move(reader_));
        } else if (ciEqLibC()(deflate, *it)) {
            RESTC_CPP_LOG_TRACE << "Adding deflate reader to " << *connection_;
            reader_ = DataReader::CreateZipReader(move(reader_));
        } else
#endif // RESTC_CPP_WITH_ZLIB
        {
            RESTC_CPP_LOG_ERROR << "Unsupported compression: '"
                << url_encode(*it)
                << "' from server on " << *connection_;
            throw NotSupportedException("Unsupported compression.");
        }
    }
}

boost::asio::const_buffers_1 ReplyImpl::GetSomeData()  {
    auto rval = reader_
        ? reader_->ReadSome()
        : boost::asio::const_buffers_1{nullptr, 0};
    CheckIfWeAreDone();
    return rval;
}

string ReplyImpl::GetBodyAsString(const size_t maxSize) {
    std::string buffer;
    if (content_length_) {
        buffer.reserve(*content_length_);
    }

    while(!IsEof()) {
        auto data = reader_->ReadSome();

        const auto buffer_size = boost::asio::buffer_size(data);
        if ((buffer.size() + buffer_size) >= maxSize) {
            throw ConstraintException(
                "Too much data for the curent buffer limit.");
        }

        buffer.append(boost::asio::buffer_cast<const char*>(data),
                      buffer_size);
    }

    ReleaseConnection();
    return buffer;
}

void ReplyImpl::CheckIfWeAreDone() {
    if (reader_ && reader_->IsEof()) {
        ReleaseConnection();
    }
}

void ReplyImpl::ReleaseConnection() {
    if (connection_ && do_close_connection_) {
        RESTC_CPP_LOG_TRACE << "Closing connection because do_close_connection_ is true: "
            << *connection_;
        if (connection_->GetSocket().IsOpen()) {
            connection_->GetSocket().Close();
        }
    }

    if (connection_) {
        RESTC_CPP_LOG_TRACE << "Releasing " << *connection_;
        connection_.reset();
        //reader_.reset();
    }
}

std::unique_ptr<ReplyImpl>
ReplyImpl::Create(Connection::ptr_t connection,
       Context& ctx,
       RestClient& owner,
       Request::Properties::ptr_t& properties) {

    return make_unique<ReplyImpl>(move(connection), ctx, owner, properties);
}

} // restc_cpp
