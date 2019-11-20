/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __mbot_status_t_hpp__
#define __mbot_status_t_hpp__

#include <string>


class mbot_status_t
{
    public:
        int64_t    utime;

        int8_t     status;

        std::string other_error_description;

    public:
        static const int8_t   STATUS_IN_PROGRESS = 0;
        static const int8_t   STATUS_COMPLETE = 1;
        static const int8_t   STATUS_GOAL_INVALID = 2;
        static const int8_t   STATUS_NO_VALID_PATH = 3;

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "mbot_status_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

int mbot_status_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int mbot_status_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int mbot_status_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t mbot_status_t::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* mbot_status_t::getTypeName()
{
    return "mbot_status_t";
}

int mbot_status_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->status, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    char* other_error_description_cstr = (char*) this->other_error_description.c_str();
    tlen = __string_encode_array(buf, offset + pos, maxlen - pos, &other_error_description_cstr, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int mbot_status_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->status, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    int32_t __other_error_description_len__;
    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &__other_error_description_len__, 1);
    if(tlen < 0) return tlen; else pos += tlen;
    if(__other_error_description_len__ > maxlen - pos) return -1;
    this->other_error_description.assign(((const char*)buf) + offset + pos, __other_error_description_len__ - 1);
    pos += __other_error_description_len__;

    return pos;
}

int mbot_status_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int8_t_encoded_array_size(NULL, 1);
    enc_size += this->other_error_description.size() + 4 + 1;
    return enc_size;
}

int64_t mbot_status_t::_computeHash(const __lcm_hash_ptr *)
{
    int64_t hash = 0xc9e14801de0f2d50LL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
