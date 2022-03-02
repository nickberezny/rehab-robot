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
 * @author Nick Berezny
 * @date 14 Jan 2022
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

G_MODULE_EXPORT void handle_run_click()
{
  printf("Run!\n");
  system("./robotController");
}

G_MODULE_EXPORT void handle_set_click()
{
  printf("Set!\n");
  //system("./robotController");
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

    GtkWidget *set_button;
    GtkWidget *run_button;

    set_button = GTK_WIDGET (gtk_builder_get_object(builder,"set_button"));
    run_button = GTK_WIDGET (gtk_builder_get_object(builder,"run_button"));

    g_signal_connect (set_button, "clicked", G_CALLBACK (handle_set_click), NULL);
    g_signal_connect (run_button, "clicked", G_CALLBACK (handle_run_click), NULL);

    //Quit the app when the window is closed.
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    // Main loop.
    gtk_main();

    return 0;
}