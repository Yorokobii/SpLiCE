#ifndef OFFSCREEN_MECACELLVIEWERPLUGIN_HPP
#define OFFSCREEN_MECACELLVIEWERPLUGIN_HPP
#include <QImage>
#include <QOpenGLFramebufferObject>
/**
 * @brief ScreenCapturePlugin creates a checkable option in the menu that enables screen
 * capture
 */

struct OffscreenPlugin {
	GLuint fbo, render_buf;

	~OffscreenPlugin(){
		GL()->glDeleteFramebuffers(1,&fbo);
		GL()->glDeleteRenderbuffers(1,&render_buf);
	}

	void saveImg(int W, int H, const QString &path) {
		std::vector<GLubyte> pixels;
		pixels.resize(3 * W * H);
		GL()->glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		GL()->glReadPixels(0, 0, W, H, GL_RGB, GL_UNSIGNED_BYTE, &pixels[0]);
		QImage img(&pixels[0], W, H, QImage::Format_RGB888);
		img.mirrored().save(path + QString("capture_") + QString::number(cap++) + ".jpg");
	}

	QString path = "./capture/";
	int cap = 0;
	int skippedFrame = 0;

	template <typename R> void preLoad(R *renderer) {

		std::cerr<<"test"<<std::endl;
		GL()->glGenFramebuffers(1,&fbo);
		std::cerr<<"test"<<std::endl;
		GL()->glGenRenderbuffers(1,&render_buf);
		std::cerr<<"test"<<std::endl;

		QDir dir;
		dir.mkdir(path);
		dir.setPath(path);
		dir.setNameFilters(QStringList() << "capture_*");
		dir.setFilter(QDir::Files);
		foreach(QString dirFile, dir.entryList())
		{
			dir.remove(dirFile);
		}
	}

	template <typename R> void preDraw(R *renderer) {
		std::cerr<<"test"<<std::endl;
		GL()->glBindRenderbuffer(GL_RENDERBUFFER, render_buf);
		GL()->glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB, renderer->getWindow()->width(), renderer->getWindow()->height());
		GL()->glBindFramebuffer(GL_DRAW_FRAMEBUFFER,fbo);
		GL()->glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, render_buf);
	}

	template <typename R> void postDraw(R *renderer) {
		// GL()->glReadBuffer(GL_COLOR_ATTACHMENT0);

		saveImg(renderer->getWindow()->width(), renderer->getWindow()->height(), path);

		// Return to onscreen rendering:
		GL()->glBindFramebuffer(GL_DRAW_FRAMEBUFFER,0);		
	}
};

#endif
