#include "restc-cpp/restc-cpp.h"
#include "restc-cpp/DataReader.h"

using namespace std;

namespace restc_cpp {


class NoBodyReaderImpl : public DataReader {
public:
    NoBodyReaderImpl() {}

    bool IsEof() const override {
        return true;
    }

    boost::asio::const_buffers_1 ReadSome() override {
        return {nullptr, 0};
    }
};

DataReader::ptr_t
DataReader::CreateNoBodyReader() {
    return make_unique<NoBodyReaderImpl>();
}


} // namespace

