#include "application/initialize.h"
#include "application/control/form/form.h"
#include "application/application.h"
#include "gameInfo.h"

struct gameForm : public form
{
	gameForm()
	{
	}
	void updateStream()
	{
	}
	virtual void keyDown(cvk& keyCode) override
	{
	}
	virtual void mouseDown(cveci2& position, cmb& button) override
	{
	}
	virtual void scroll(cveci2& position, cint& scrollDelta) override
	{
	}
	virtual void render(cveci2& position, const texture& renderTarget) override
	{
	}
};
gameForm* mainForm = new gameForm();
int main(int argc, char* argv[])
{
	// execute this function before you do anything,
	initialize();
	return application(mainForm, gameName).run();
}