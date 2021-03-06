package org.szte.wsn.downloader2;

/**
 * This class is automatically generated by mig. DO NOT EDIT THIS FILE.
 * This class implements a Java interface to the 'TimeMsg'
 * message type.
 */

public class TimeMsg extends net.tinyos.message.Message {

    /** The default size of this message type in bytes. */
    public static final int DEFAULT_MESSAGE_SIZE = 14;

    /** The Active Message type associated with this message. */
    public static final int AM_TYPE = 1;

    /** Create a new TimeMsg of size 14. */
    public TimeMsg() {
        super(DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /** Create a new TimeMsg of the given data_length. */
    public TimeMsg(int data_length) {
        super(data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TimeMsg with the given data_length
     * and base offset.
     */
    public TimeMsg(int data_length, int base_offset) {
        super(data_length, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TimeMsg using the given byte array
     * as backing store.
     */
    public TimeMsg(byte[] data) {
        super(data);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TimeMsg using the given byte array
     * as backing store, with the given base offset.
     */
    public TimeMsg(byte[] data, int base_offset) {
        super(data, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TimeMsg using the given byte array
     * as backing store, with the given base offset and data length.
     */
    public TimeMsg(byte[] data, int base_offset, int data_length) {
        super(data, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TimeMsg embedded in the given message
     * at the given base offset.
     */
    public TimeMsg(net.tinyos.message.Message msg, int base_offset) {
        super(msg, base_offset, DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new TimeMsg embedded in the given message
     * at the given base offset and length.
     */
    public TimeMsg(net.tinyos.message.Message msg, int base_offset, int data_length) {
        super(msg, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
    /* Return a String representation of this message. Includes the
     * message type name and the non-indexed field values.
     */
    public String toString() {
      String s = "Message <TimeMsg> \n";
      try {
        s += "  [remoteTime=0x"+Long.toHexString(get_remoteTime())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [localTime=0x"+Long.toHexString(get_localTime())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [sendTime=0x"+Long.toHexString(get_sendTime())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [bootCount=0x"+Long.toHexString(get_bootCount())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      return s;
    }

    // Message-type-specific access methods appear below.

    /////////////////////////////////////////////////////////
    // Accessor methods for field: remoteTime
    //   Field type: long, unsigned
    //   Offset (bits): 0
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'remoteTime' is signed (false).
     */
    public static boolean isSigned_remoteTime() {
        return false;
    }

    /**
     * Return whether the field 'remoteTime' is an array (false).
     */
    public static boolean isArray_remoteTime() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'remoteTime'
     */
    public static int offset_remoteTime() {
        return (0 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'remoteTime'
     */
    public static int offsetBits_remoteTime() {
        return 0;
    }

    /**
     * Return the value (as a long) of the field 'remoteTime'
     */
    public long get_remoteTime() {
        return (long)getUIntBEElement(offsetBits_remoteTime(), 32);
    }

    /**
     * Set the value of the field 'remoteTime'
     */
    public void set_remoteTime(long value) {
        setUIntBEElement(offsetBits_remoteTime(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'remoteTime'
     */
    public static int size_remoteTime() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'remoteTime'
     */
    public static int sizeBits_remoteTime() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: localTime
    //   Field type: long, unsigned
    //   Offset (bits): 32
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'localTime' is signed (false).
     */
    public static boolean isSigned_localTime() {
        return false;
    }

    /**
     * Return whether the field 'localTime' is an array (false).
     */
    public static boolean isArray_localTime() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'localTime'
     */
    public static int offset_localTime() {
        return (32 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'localTime'
     */
    public static int offsetBits_localTime() {
        return 32;
    }

    /**
     * Return the value (as a long) of the field 'localTime'
     */
    public long get_localTime() {
        return (long)getUIntBEElement(offsetBits_localTime(), 32);
    }

    /**
     * Set the value of the field 'localTime'
     */
    public void set_localTime(long value) {
        setUIntBEElement(offsetBits_localTime(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'localTime'
     */
    public static int size_localTime() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'localTime'
     */
    public static int sizeBits_localTime() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: sendTime
    //   Field type: long, unsigned
    //   Offset (bits): 64
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'sendTime' is signed (false).
     */
    public static boolean isSigned_sendTime() {
        return false;
    }

    /**
     * Return whether the field 'sendTime' is an array (false).
     */
    public static boolean isArray_sendTime() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'sendTime'
     */
    public static int offset_sendTime() {
        return (64 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'sendTime'
     */
    public static int offsetBits_sendTime() {
        return 64;
    }

    /**
     * Return the value (as a long) of the field 'sendTime'
     */
    public long get_sendTime() {
        return (long)getUIntBEElement(offsetBits_sendTime(), 32);
    }

    /**
     * Set the value of the field 'sendTime'
     */
    public void set_sendTime(long value) {
        setUIntBEElement(offsetBits_sendTime(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'sendTime'
     */
    public static int size_sendTime() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'sendTime'
     */
    public static int sizeBits_sendTime() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: bootCount
    //   Field type: int, unsigned
    //   Offset (bits): 96
    //   Size (bits): 16
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'bootCount' is signed (false).
     */
    public static boolean isSigned_bootCount() {
        return false;
    }

    /**
     * Return whether the field 'bootCount' is an array (false).
     */
    public static boolean isArray_bootCount() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'bootCount'
     */
    public static int offset_bootCount() {
        return (96 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'bootCount'
     */
    public static int offsetBits_bootCount() {
        return 96;
    }

    /**
     * Return the value (as a int) of the field 'bootCount'
     */
    public int get_bootCount() {
        return (int)getUIntBEElement(offsetBits_bootCount(), 16);
    }

    /**
     * Set the value of the field 'bootCount'
     */
    public void set_bootCount(int value) {
        setUIntBEElement(offsetBits_bootCount(), 16, value);
    }

    /**
     * Return the size, in bytes, of the field 'bootCount'
     */
    public static int size_bootCount() {
        return (16 / 8);
    }

    /**
     * Return the size, in bits, of the field 'bootCount'
     */
    public static int sizeBits_bootCount() {
        return 16;
    }

}
