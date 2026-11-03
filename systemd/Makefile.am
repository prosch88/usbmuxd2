edit = \
	$(SED) -r \
	-e 's|@sbindir[@]|$(sbindir)|g' \
	-e 's|@localstatedir[@]|$(localstatedir)|g' \
	< $< > $@ || rm $@

if WANT_SYSTEMD

systemdsystemunit_DATA = \
	usbmuxd.service

systemduserunit_DATA = \
	usbmuxd-notify.service

usbmuxd.service: usbmuxd.service.in
	$(edit)

usbmuxd-notify.service: usbmuxd-notify.service.in
	$(edit)

EXTRA_DIST = \
	usbmuxd.service.in \
	usbmuxd-notify.service.in

CLEANFILES = \
	usbmuxd.service \
	usbmuxd-notify.service

endif
