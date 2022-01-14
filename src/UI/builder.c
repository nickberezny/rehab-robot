/****************************************************************************
 * Copyright (C) 2012 by Matteo Franchin                                    *
 *                                                                          *
 * This file is part of Box.                                                *
 *                                                                          *
 *   Box is free software: you can redistribute it and/or modify it         *
 *   under the terms of the GNU Lesser General Public License as published  *
 *   by the Free Software Foundation, either version 3 of the License, or   *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   Box is distributed in the hope that it will be useful,                 *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU Lesser General Public License for more details.                    *
 *                                                                          *
 *   You should have received a copy of the GNU Lesser General Public       *
 *   License along with Box.  If not, see <http://www.gnu.org/licenses/>.   *
 ****************************************************************************/


/**
 * @file builder.c
 * @author My Self
 * @date 9 Sep 2012
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 * @see http://www.stack.nl/~dimitri/doxygen/docblocks.html
 * @see http://www.stack.nl/~dimitri/doxygen/commands.html
 */

#include <gtk/gtk.h>
#include <stdio.h>

G_MODULE_EXPORT void handle_button_click(GtkButton* button,
                                         gpointer user_data)
{
  printf("click: %p, %X\n", button, *(int*)user_data);
}

int main(int argc, char** argv)
{
	gtk_init(&argc, &argv);

    // Create a builder object that will load the file.
    GtkBuilder* builder = gtk_builder_new();

    // Load the XML from a file.
    gtk_builder_add_from_file(builder, "src/UI/test.glade", NULL);

    // Get the object called 'main_window' from the file and show it.
    GObject* window = gtk_builder_get_object(builder, "main_window");
    gtk_widget_show(GTK_WIDGET(window));

    // Connect the signal handlers defined in the glade file.
    // (Note: if you're looking for the c++ way to do this, there's no
    // support for binding C++ signal handlers. You must use 'extern
    // "C"' functions as handlers.)
    //int my_user_data = 0xDEADBEEF;
    //gtk_builder_connect_signals(builder, &my_user_data);

    // Quit the app when the window is closed.
    //g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    // Main loop.
    gtk_main();

    return 0;
}