flash:
	@JLinkExe -commandfile flash.jlink

tags:
	@cscope -R -b
	@ctags -R
