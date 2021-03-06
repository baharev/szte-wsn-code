package org.szte.wsn.downloader2;

/**
 * This class is automatically generated by mig. DO NOT EDIT THIS FILE.
 * This class implements a Java interface to the 'CommandMsg'
 * message type.
 */

public class CommandMsg extends net.tinyos.message.Message {

	public static final short COMMAND_PING=0x00;
	public static final short COMMAND_ERASE=0x01;
	public static final short COMMAND_STOPSEND=0x02;
	
	public static final short COMMAND_SYNC=0x11;
	public static final short COMMAND_STARTSTOP=0x22;
	public static final short COMMAND_SETTIMESYNCINTERVAL=0x33;
	public static final short COMMAND_SETMEASUREINTERVAL=0x44;
	public static final short COMMAND_SETTIMESYNCSAVEPOINTS=0x55;
	public static final short COMMAND_LEDSON=0xa0;
	public static final short COMMAND_SHUTDOWN=0xa1;
	public static final short COMMAND_SENDTIMESYNC=0xa2;	
	
	
    /** The default size of this message type in bytes. */
    public static final int DEFAULT_MESSAGE_SIZE = 1;

    /** The Active Message type associated with this message. */
    public static final int AM_TYPE = 98;

    /** Create a new CommandMsg of size 1. */
    public CommandMsg() {
        super(DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /** Create a new CommandMsg of the given data_length. */
    public CommandMsg(int data_length) {
        super(data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CommandMsg with the given data_length
     * and base offset.
     */
    public CommandMsg(int data_length, int base_offset) {
        super(data_length, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CommandMsg using the given byte array
     * as backing store.
     */
    public CommandMsg(byte[] data) {
        super(data);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CommandMsg using the given byte array
     * as backing store, with the given base offset.
     */
    public CommandMsg(byte[] data, int base_offset) {
        super(data, base_offset);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CommandMsg using the given byte array
     * as backing store, with the given base offset and data length.
     */
    public CommandMsg(byte[] data, int base_offset, int data_length) {
        super(data, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CommandMsg embedded in the given message
     * at the given base offset.
     */
    public CommandMsg(net.tinyos.message.Message msg, int base_offset) {
        super(msg, base_offset, DEFAULT_MESSAGE_SIZE);
        amTypeSet(AM_TYPE);
    }

    /**
     * Create a new CommandMsg embedded in the given message
     * at the given base offset and length.
     */
    public CommandMsg(net.tinyos.message.Message msg, int base_offset, int data_length) {
        super(msg, base_offset, data_length);
        amTypeSet(AM_TYPE);
    }

    /**
    /* Return a String representation of this message. Includes the
     * message type name and the non-indexed field values.
     */
    public String toString() {
      String s = "Message <CommandMsg> \n";
      try {
        s += "  [cmd=0x"+Long.toHexString(get_cmd())+"]\n";
      } catch (ArrayIndexOutOfBoundsException aioobe) { /* Skip field */ }
      return s;
    }

    // Message-type-specific access methods appear below.

    /////////////////////////////////////////////////////////
    // Accessor methods for field: cmd
    //   Field type: short, unsigned
    //   Offset (bits): 0
    //   Size (bits): 8
    /////////////////////////////////////////////////////////

    /**
     * Return whether the field 'cmd' is signed (false).
     */
    public static boolean isSigned_cmd() {
        return false;
    }

    /**
     * Return whether the field 'cmd' is an array (false).
     */
    public static boolean isArray_cmd() {
        return false;
    }

    /**
     * Return the offset (in bytes) of the field 'cmd'
     */
    public static int offset_cmd() {
        return (0 / 8);
    }

    /**
     * Return the offset (in bits) of the field 'cmd'
     */
    public static int offsetBits_cmd() {
        return 0;
    }

    /**
     * Return the value (as a short) of the field 'cmd'
     */
    public short get_cmd() {
        return (short)getUIntBEElement(offsetBits_cmd(), 8);
    }

    /**
     * Set the value of the field 'cmd'
     */
    public void set_cmd(short value) {
        setUIntBEElement(offsetBits_cmd(), 8, value);
    }

    /**
     * Return the size, in bytes, of the field 'cmd'
     */
    public static int size_cmd() {
        return (8 / 8);
    }

    /**
     * Return the size, in bits, of the field 'cmd'
     */
    public static int sizeBits_cmd() {
        return 8;
    }

}
