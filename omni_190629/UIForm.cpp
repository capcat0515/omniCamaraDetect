#include "UIForm.h"

using namespace omni190629;

[STAThreadAttribute]

int main(cli::array<System::String ^> ^args)
{
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);

	Application::Run(gcnew UIForm());


	return 0;
}