pub fn u8_to_decimal_str<'a>(mut n: u8, buf: &'a mut [u8; 3]) -> &'a str {
    // We'll write the decimal digits of n into buf from the end to the front.
    // Example: if n = 42, we end up writing '4', '2' into the last two spots.
    // Then we return the substring of buf that contains these digits.

    // Max decimal length of a u8 is 3 (e.g. 255)
    let mut i = buf.len();

    // Special case for zero:
    if n == 0 {
        i -= 1;
        buf[i] = b'0';
        return unsafe { core::str::from_utf8_unchecked(&buf[i..]) };
    }

    while n > 0 {
        i -= 1;
        buf[i] = b'0' + (n % 10);
        n /= 10;
    }

    // Now buf[i..] has the ASCII digits. We can convert this to a &str.
    // Safe because we only wrote ASCII digits (0x30..0x39).
    unsafe { core::str::from_utf8_unchecked(&buf[i..]) }
}
