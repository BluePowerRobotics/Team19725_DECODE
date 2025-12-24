package org.firstinspires.ftc.teamcode.controllers;

import java.util.Locale;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class InstanceTelemetry implements Telemetry {
    private static InstanceTelemetry instance;
    private Telemetry telemetry;
    private InstanceTelemetry(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public static InstanceTelemetry init(Telemetry telemetry){
        instance = new InstanceTelemetry(telemetry);
        return instance;
    }

    public static InstanceTelemetry getTelemetry() {
        if (instance == null) {
            instance = new InstanceTelemetry(new Telemetry() {
                @Override
                public Item addData(String caption, String format, Object... args) {
                    return null;
                }

                @Override
                public Item addData(String caption, Object value) {
                    return null;
                }

                @Override
                public <T> Item addData(String caption, Func<T> valueProducer) {
                    return null;
                }

                @Override
                public <T> Item addData(String caption, String format, Func<T> valueProducer) {
                    return null;
                }

                @Override
                public boolean removeItem(Item item) {
                    return false;
                }

                @Override
                public void clear() {

                }

                @Override
                public void clearAll() {

                }

                @Override
                public Object addAction(Runnable action) {
                    return null;
                }

                @Override
                public boolean removeAction(Object token) {
                    return false;
                }

                @Override
                public void speak(String text) {

                }

                @Override
                public void speak(String text, String languageCode, String countryCode) {

                }

                @Override
                public boolean update() {
                    return false;
                }

                @Override
                public Line addLine() {
                    return null;
                }

                @Override
                public Line addLine(String lineCaption) {
                    return null;
                }

                @Override
                public boolean removeLine(Line line) {
                    return false;
                }

                @Override
                public boolean isAutoClear() {
                    return false;
                }

                @Override
                public void setAutoClear(boolean autoClear) {

                }

                @Override
                public int getMsTransmissionInterval() {
                    return 0;
                }

                @Override
                public void setMsTransmissionInterval(int msTransmissionInterval) {

                }

                @Override
                public String getItemSeparator() {
                    return "";
                }

                @Override
                public void setItemSeparator(String itemSeparator) {

                }

                @Override
                public String getCaptionValueSeparator() {
                    return "";
                }

                @Override
                public void setCaptionValueSeparator(String captionValueSeparator) {

                }

                @Override
                public void setDisplayFormat(DisplayFormat displayFormat) {

                }

                @Override
                public Log log() {
                    return null;
                }
            });
        }
        return instance;
    }

    /**
     * Adds an item to the end of the telemetry being built for driver station display. The value shown
     * will be the result of calling {@link String#format(Locale, String, Object...) String.format()}
     * with the indicated format and arguments. The caption and value are shown on the driver station
     * separated by the {@link #getCaptionValueSeparator() caption value separator}. The item
     * is removed if {@link #clear()} or {@link #clearAll()} is called.
     *
     * @param caption the caption to use
     * @param format  the string by which the arguments are to be formatted
     * @param args    the arguments to format
     * @return an {@link Item} that can be used to update the value or append further {@link Item}s
     * @see #addData(String, Object)
     * @see #addData(String, Func)
     */
    @Override
    public Item addData(String caption, String format, Object... args) {
        return telemetry.addData(caption, format, args);
    }

    /**
     * Adds an item to the end if the telemetry being built for driver station display. The value shown
     * will be the result of calling {@link Object#toString() toString()} on the provided value
     * object. The caption and value are shown on the driver station separated by the {@link
     * #getCaptionValueSeparator() caption value separator}. The item is removed if {@link #clear()}
     * or {@link #clearAll()} is called.
     *
     * @param caption the caption to use
     * @param value   the value to display
     * @return an {@link Item} that can be used to update the value or append further {@link Item}s
     * @see #addData(String, String, Object...)
     * @see #addData(String, Func)
     */
    @Override
    public Item addData(String caption, Object value) {
        return telemetry.addData(caption,value);
    }

    /**
     * Adds an item to the end of the telemetry being built for driver station display. The value shown
     * will be the result of calling {@link Object#toString() toString()} on the object which is
     * returned from invoking valueProducer.{@link Func#value()} value()}. The caption and value are
     * shown on the driver station separated by the {@link #getCaptionValueSeparator() caption value
     * separator}. The item is removed if {@link #clearAll()} is called, but <em>not</em> if
     * {@link #clear()} is called.
     *
     * <p>The valueProducer is evaluated only if actual transmission to the driver station
     * is to occur. This is important, as it provides a means of displaying telemetry which
     * is relatively expensive to evaluate while avoiding computation or delay on evaluations
     * which won't be transmitted due to transmission interval throttling.</p>
     *
     * @param caption       the caption to use
     * @param valueProducer the object which will provide the value to display
     * @return an {@link Item} that can be used to update the value or append further {@link Item}s
     * @see #addData(String, String, Object...)
     * @see #addData(String, Object)
     * @see #addData(String, String, Func)
     * @see #getMsTransmissionInterval()
     */
    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return telemetry.addData(caption,valueProducer);
    }

    /**
     * Adds an item to the end of the telemetry being built for driver station display. The value shown
     * will be the result of calling {@link String#format} on the object which is returned from invoking
     * valueProducer.{@link Func#value()} value()}. The caption and value are shown on the driver station
     * separated by the {@link #getCaptionValueSeparator() caption value separator}. The item is removed
     * if {@link #clearAll()} is called, but <em>not</em> if {@link #clear()} is called.
     *
     * <p>The valueProducer is evaluated only if actual transmission to the driver station
     * is to occur. This is important, as it provides a means of displaying telemetry which
     * is relatively expensive to evaluate while avoiding computation or delay on evaluations
     * which won't be transmitted due to transmission interval throttling.</p>
     *
     * @param caption       the caption to use
     * @param format
     * @param valueProducer the object which will provide the value to display
     * @return an {@link Item} that can be used to update the value or append further {@link Item}s
     * @see #addData(String, String, Object...)
     * @see #addData(String, Object)
     * @see #addData(String, Func)
     * @see #getMsTransmissionInterval()
     */
    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return telemetry.addData(caption,format,valueProducer);
    }

    /**
     * Removes an item from the receiver telemetry, if present.
     *
     * @param item the item to remove
     * @return true if any change was made to the receive (ie: the item was present); false otherwise
     */
    @Override
    public boolean removeItem(Item item) {
        return telemetry.removeItem(item);
    }

    /**
     * Removes all items from the receiver whose value is not to be retained.
     *
     * @see Item#setRetained(Boolean)
     * @see Item#isRetained()
     * @see #clearAll()
     * @see #addData(String, Func)
     */
    @Override
    public void clear() {
        telemetry.clear();
    }

    /**
     * Removes <em>all</em> items, lines, and actions from the receiver
     *
     * @see #clear()
     */
    @Override
    public void clearAll() {
        telemetry.clearAll();
    }

    /**
     * In addition to items and lines, a telemetry may also contain a list of actions.
     * When the telemetry is to be updated, these actions are evaluated before the telemetry
     * lines are composed just prior to transmission. A typical use of such actions is to
     * initialize some state variable, parts of which are subsequently displayed in items.
     * This can help avoid needless re-evaluation.
     *
     * <p>Actions are cleared with {@link #clearAll()}, and can be removed with {@link
     * #removeAction(Object) removeAction()}.</p>
     *
     * @param action the action to execute before composing the lines telemetry
     * @return a token by which the action can be later removed.
     * @see #addData(String, Object)
     * @see #removeAction(Object)
     * @see #addLine()
     * @see #update()
     */
    @Override
    public Object addAction(Runnable action) {
        return telemetry.addAction(action);
    }

    /**
     * Removes a previously added action from the receiver.
     *
     * @param token the token previously returned from {@link #addAction(Runnable) addAction()}.
     * @return whether any change was made to the receiver
     */
    @Override
    public boolean removeAction(Object token) {
        return telemetry.removeAction(token);
    }

    /**
     * Directs the Driver Station device to speak the given text using TextToSpeech functionality,
     * with the same language and country codes that were previously used, or the default language
     * and country.
     *
     * @param text the text to be spoken
     */
    @Override
    public void speak(String text) {
        telemetry.speak(text);
    }

    /**
     * Directs the Driver Station device to speak the given text using TextToSpeech functionality,
     * with the given language and country codes.
     *
     * @param text         the text to be spoken
     * @param languageCode an ISO 639 alpha-2 or alpha-3 language code, or a language subtag up to
     *                     8 characters in length
     * @param countryCode  an ISO 3166 alpha-2 country code, or a UN M.49 numeric-3 area code
     */
    @Override
    public void speak(String text, String languageCode, String countryCode) {
        telemetry.speak(text,languageCode,countryCode);
    }

    /**
     * Sends the receiver {@link Telemetry} to the driver station if more than the {@link #getMsTransmissionInterval()
     * transmission interval} has elapsed since the last transmission, or schedules the transmission
     * of the receiver should no subsequent {@link Telemetry} state be scheduled for transmission before
     * the {@link #getMsTransmissionInterval() transmission interval} expires.
     *
     * @return whether a transmission to the driver station occurred or not
     */
    @Override
    public boolean update() {
        return telemetry.update();
    }

    /**
     * Creates and returns a new line in the receiver {@link Telemetry}.
     *
     * @return a new line in the receiver {@link Telemetry}
     */
    @Override
    public Line addLine() {
        return telemetry.addLine();
    }

    /**
     * Creates and returns a new line in the receiver {@link Telemetry}.
     *
     * @param lineCaption the caption for the line
     * @return a new line in the receiver {@link Telemetry}
     */
    @Override
    public Line addLine(String lineCaption) {
        return telemetry.addLine(lineCaption);
    }

    /**
     * Removes a line from the receiver telemetry, if present.
     *
     * @param line the line to be removed
     * @return whether any change was made to the receiver
     */
    @Override
    public boolean removeLine(Line line) {
        return telemetry.removeLine(line);
    }

    /**
     * Answers whether {@link #clear()} is automatically called after each call to {@link #update()}.
     *
     * @return whether {@link #clear()} is automatically called after each call to {@link #update()}.
     * @see #setAutoClear(boolean)
     */
    @Override
    public boolean isAutoClear() {
        return telemetry.isAutoClear();
    }

    /**
     * Sets whether {@link #clear()} is automatically called after each call to {@link #update()}.
     *
     * @param autoClear if true, {@link #clear()} is automatically called after each call to {@link #update()}.
     */
    @Override
    public void setAutoClear(boolean autoClear) {
        telemetry.setAutoClear(autoClear);
    }

    /**
     * Returns the minimum interval between {@link Telemetry} transmissions from the robot controller
     * to the driver station
     *
     * @return the minimum interval between {@link Telemetry} transmissions from the robot controller to the diver station
     * @see #setMsTransmissionInterval(int)
     */
    @Override
    public int getMsTransmissionInterval() {
        return telemetry.getMsTransmissionInterval();
    }

    /**
     * Sets the minimum interval between {@link Telemetry} transmissions from the robot controller
     * to the driver station.
     *
     * @param msTransmissionInterval the minimum interval between {@link Telemetry} transmissions
     *                               from the robot controller to the driver station
     * @see #getMsTransmissionInterval()
     */
    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        telemetry.setMsTransmissionInterval(msTransmissionInterval);
    }

    /**
     * Returns the string which is used to separate {@link Item}s contained within a line. The default
     * separator is " | ".
     *
     * @return the string which is use to separate {@link Item}s contained within a line.
     * @see #setItemSeparator(String)
     * @see #addLine()
     */
    @Override
    public String getItemSeparator() {
        return telemetry.getItemSeparator();
    }

    /**
     * @param itemSeparator
     * @see #setItemSeparator(String)
     */
    @Override
    public void setItemSeparator(String itemSeparator) {
        telemetry.setItemSeparator(itemSeparator);
    }

    /**
     * Returns the string which is used to separate caption from value within a {@link Telemetry}
     * {@link Item}. The default separator is " : ";
     *
     * @return the string which is used to separate caption from value within a {@link Telemetry} {@link Item}.
     */
    @Override
    public String getCaptionValueSeparator() {
        return telemetry.getCaptionValueSeparator();
    }

    /**
     * @param captionValueSeparator
     * @see #getCaptionValueSeparator()
     */
    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        telemetry.setCaptionValueSeparator(captionValueSeparator);
    }

    /**
     * Sets the telemetry display format on the Driver Station. See the comments on {@link DisplayFormat}.
     *
     * @param displayFormat the telemetry display format the Driver Station should use
     */
    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        telemetry.setDisplayFormat(displayFormat);
    }

    /**
     * Returns the log of this {@link Telemetry} to which log entries may be appended.
     *
     * @return the log of this {@link Telemetry} to which log entries may be appended.
     * @see Log#addData(String, Object)
     */
    @Override
    public Log log() {
        return telemetry.log();
    }
}
