all: graphview

CFLAGS=-ansi -D_DONT_USE_STACK_GUARD -I .
OBJFILES=Main.o PNGLoad.o CallStack.o DCT.o Dir.o Jpeg.o NewFix.o StlUtils.o Tga.o Thread.o Timer.o Vector.o BufferObjects.o FrameBuffer.o GLES20Emu.o Shader.o Texture.o TextureUtil.o Transform.o FontTexture2.o BitmapFont2.o UniConv.o UniBlocks.o

graphview: $(OBJFILES)
	g++ $(CFLAGS) -o graphview $(OBJFILES) -lGL -lglut -lpng -lpthread -lX11
# note that if the build fails, just try putting the linker flags (the ones starting with -l) at the end of the line

%.o: %.cpp
	g++ $(CFLAGS) -c $< -o $@

%.o: ../UberLame_src/%.cpp
	g++ $(CFLAGS) -c $< -o $@

%.o: ../UberLame_src/gles2/%.cpp
	g++ $(CFLAGS) -c $< -o $@

%.o: ../UberLame_src/iface/%.cpp
	g++ $(CFLAGS) -c $< -o $@

clean:
	rm -f *.o graphview

test:
	./graphview
