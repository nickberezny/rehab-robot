/**
 * @file UI.c
 * @author Nick Berezny
 * @date 18 Jan 2022
 * @Controller thread 
 *
 */

#include <gtk/gtk.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "./include/UI.h"
#include "./include/Communication.h"

int fd;

G_MODULE_EXPORT void handle_run_click()
{
  printf("Run!\n");
  //system("./robotController");
}

G_MODULE_EXPORT void handle_set_click()
{
    
    write(fd, "Hi NEW", sizeof("Hi NEW"));
    printf("%d Set!\n",fd);
    close(fd);

    /* remove the FIFO */
    unlink("/tmp/myfifo");
    

}


int mainUI(int argc, char** argv)
{
    printf("Starting UI...\n");
    char * myfifo = "/tmp/myfifo";

    /* create the FIFO (named pipe) */
    mkfifo(myfifo, 0666);
    /* write "Hi" to the FIFO */
    fd = open(myfifo, O_WRONLY);
    //write(fd, "Hi NEW", sizeof("Hi NEW"));
    printf("%d Set!\n", fd);
    

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