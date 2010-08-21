package org.szte.wsn.downloader2;

/**
 * This class is automatically generated by mig. DO NOT EDIT THIS FILE.
 * This class implements a Java interface to the 'CtrlMsg'
 * message type.
 */

public class CtrlMsg extends net.tinyos.message.Message {

    /** The default size of this message type in bytes. */
    public static final int DEFAULT_MESSAGE_SIZE = 8;

    /** The Active Message type associated with this message. */
    public static final int AM_TYPE = 96;

    /** Create a new CtrlMsg of size 8. */
    public CtrlMsg() {
        super(DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /** Create a new CtrlMsg of the given data_length. */
    public CtrlMsg(int data_length) {
        super(data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CtrlMsg with the given data_length
     * and base offset.
     */
    public CtrlMsg(int data_length, int base_offset) {
        super(data_length, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CtrlMsg using the given byte array
     * as backing store.
     */
    public CtrlMsg(byte[] data) {
        super(data);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CtrlMsg using the given byte array
     * as backing store, with the given base offset.
     */
    public CtrlMsg(byte[] data, int base_offset) {
        super(data, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CtrlMsg using the given byte array
     * as backing store, with the given base offset and data length.
     */
    public CtrlMsg(byte[] data, int base_offset, int data_length) {
        super(data, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CtrlMsg embedded in the given message
     * at the given base offset.
     */
    public CtrlMsg(net.tinyos.message.Message msg, int base_offset) {
        super(msg, base_offset, DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CtrlMsg embedded in the given message
     * at the given base offset and length.
     */
    public CtrlMsg(net.tinyos.message.Message msg, int base_offset, int data_length) {
        super(msg, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
    /* Return a String representation of this message. Includes the
     * message type name and the non-indexed field values.
     */
    public String toString() {
      String s = "Message <CtrlMsg> \n";
      try {
        s += "  [min_address=0x"+Long.toHexString(get_min_address())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      try {
        s += "  [max_address=0x"+Long.toHexString(get_max_address())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      return s;
    }

    // Message-type-specific access methods appear below.

    /////////////////////////////////////////////////////////
    // Accessor methods for field: min_address
    //   Field type: long, unsigned
    //   Offset (bits): 0
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'min_address' is signed (false).
     */
    public static boolean isSigned_min_address() {
        return false;
    }

    /**
     * Return whether the field 'min_address' is an array (false).
     */
    public static boolean isArray_min_address() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'min_address'
     */
    public static int offset_min_address() {
        return (0 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'min_address'
     */
    public static int offsetBits_min_address() {
        return 0;
    }

    /**
     * Return the value (as a long) of the field 'min_address'
     */
    public long get_min_address() {
        return (long)getUIntBEElement(offsetBits_min_address(), 32);
    }

    /**
     * Set the value of the field 'min_address'
     */
    public void set_min_address(long value) {
        setUIntBEElement(offsetBits_min_address(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'min_address'
     */
    public static int size_min_address() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'min_address'
     */
    public static int sizeBits_min_address() {
        return 32;
    }

    /////////////////////////////////////////////////////////
    // Accessor methods for field: max_address
    //   Field type: long, unsigned
    //   Offset (bits): 32
    //   Size (bits): 32
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'max_address' is signed (false).
     */
    public static boolean isSigned_max_address() {
        return false;
    }

    /**
     * Return whether the field 'max_address' is an array (false).
     */
    public static boolean isArray_max_address() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'max_address'
     */
    public static int offset_max_address() {
        return (32 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'max_address'
     */
    public static int offsetBits_max_address() {
        return 32;
    }

    /**
     * Return the value (as a long) of the field 'max_address'
     */
    public long get_max_address() {
        return (long)getUIntBEElement(offsetBits_max_address(), 32);
    }

    /**
     * Set the value of the field 'max_address'
     */
    public void set_max_address(long value) {
        setUIntBEElement(offsetBits_max_address(), 32, value);
    }

    /**
     * Return the size, in bytes, of the field 'max_address'
     */
    public static int size_max_address() {
        return (32 / 8);
    }

    /**
     * Return the size, in bits, of the field 'max_address'
     */
    public static int sizeBits_max_address() {
        return 32;
    }

}