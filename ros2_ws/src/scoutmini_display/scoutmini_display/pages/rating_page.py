import csv
import os
from datetime import datetime
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QLabel,
    QPushButton,
    QHBoxLayout,
    QTextEdit,
    QSizePolicy
)


class RatingPage(QWidget):
    """Page where users rate how helpful the robot was."""

    feedback_submitted = pyqtSignal(bool,int,str)

    def __init__(self):
        super().__init__()

        self.setSizePolicy(
            QSizePolicy.Ignored,
            QSizePolicy.Ignored
        )
        self.selected_rating = 0
        self.reached_location = None
        self.help_rating = 0
        self.star_buttons = []

        layout = QVBoxLayout(self)
        layout.setSpacing(16)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addStretch()

        self.feedback_file = os.path.expanduser(
            "~/robot_feedback.csv"
        )

        # Creating the Title of the page
        title = QLabel("Navigation Feedback")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet(
            "font-size: 40px; font-weight: bold;"
        )
        layout.addWidget(title)

        #First question being asked
        location_question = QLabel(
            "Did you get to your location?"
        )
        location_question.setAlignment(Qt.AlignCenter)
        location_question.setStyleSheet(
            "font-size: 20px;"
        )
        layout.addWidget(location_question)

        location_buttons = QHBoxLayout()

        # Adding two answer choices for easy and quick responses 
        yes_button = QPushButton("Yes")
        no_button = QPushButton("No")

        for button in [yes_button, no_button]:
            button.setStyleSheet(
                "font-size: 24px; font-weight: bold;"
            )
        # Location is correct 
        yes_button.clicked.connect(
            lambda: self.set_location_result(True)
        )
        # Location is not correct
        no_button.clicked.connect(
            lambda: self.set_location_result(False)
        )

        location_buttons.addWidget(yes_button)
        location_buttons.addWidget(no_button) 

        layout.addLayout(location_buttons)

        self.location_status = QLabel(
            "Please select an option"
        )
        self.location_status.setAlignment(Qt.AlignCenter)
        self.location_status.setStyleSheet(
            "font-size: 15px;"
        )
        layout.addWidget(self.location_status)

        # Question 2 : Rating how much the robot helped you out of 5 
        help_question = QLabel(
            "How much did the robot help you?"
        )
        help_question.setAlignment(Qt.AlignCenter)
        help_question.setStyleSheet(
            "font-size: 20px;"
        )
        layout.addWidget(help_question)

        #Buttons that will be used to rate how helpful the robot was
        rating_buttons = QHBoxLayout()

        for rating in range(1, 6):
            button = QPushButton(f"☆")
            button.setStyleSheet(
                "font-size: 40px; font-weight: bold; border: none;"
            )

            button.clicked.connect( 
                lambda checked=False, value=rating:
                self.set_rating(value)
            )

            self.star_buttons.append(button)
            rating_buttons.addWidget(button)
        
        layout.addLayout(rating_buttons)
        
        self.rating_status = QLabel(
            "Select a rating from 1-5"
        )
        self.rating_status.setAlignment(Qt.AlignCenter)
        self.rating_status.setStyleSheet(
            "font-size: 15px;"
        )
        layout.addWidget(self.rating_status)

        #An option for people to add comments
        comment_label = QLabel(
            "Comments (optional) :"
        )
        comment_label.setStyleSheet(
            "font-size: 20px;"
        )
        layout.addWidget(comment_label)

        self.comment_box = QTextEdit()
        layout.addWidget(self.comment_box)

        # The button for people to submit their comments and overall feedback
        submit = QPushButton(
            "Submit Feedback"
        )

        submit.setStyleSheet(
            "font-size: 28px; font-weight: bold;"
        )

        submit.clicked.connect(
            self.submit_feedback
        )

        layout.addWidget(submit)

    #Saves the users answer and updates the screen to show the selected answer
    def set_location_result(self, reached):
        self.reached_location = reached

        if reached:
            self.location_status.setText(
                "Destination reached: Yes"
            )
        else:
            self.location_status.setText(
                "Destination reached: No"
            )
    #Saves the users rating and updates the display to show the selected rating
    def set_rating(self, rating):
        self.help_rating = rating

        for i, button in enumerate(self.star_buttons) :
            if i < rating:
                button.setText("★") #Filled in star
            else:
                button.setText("☆") #Empty star

        self.rating_status.setText(
            f"Robot helpfulness: {rating}/5"
        )

    #Checks that they answered both questions and sends the collected data
    def submit_feedback(self):
        if self.reached_location is None:
            self.location_status.setText(
                "Please answer if you reached your location"
            )
            return

        if self.help_rating == 0:
            self.rating_status.setText(
                "Please select a helpfulness rating"
            )
            return

        comment = self.comment_box.toPlainText()
        
        self.save_feedback(
            self.reached_location,
            self.help_rating,
            comment
        )

        self.feedback_submitted.emit(
            self.reached_location,
            self.help_rating,
            comment
        )

        self.location_status.setText(
            "Thank you for your feedback!"
        )

        self.feedback_submitted.emit(
            self.reached_location,
            self.help_rating,
            comment
        )  

    # Saves all information submitted by user in a csv file
    def save_feedback(self, reached_location, rating, comment):
        file_exists = os.path.exists(self.feedback_file)

        with open(self.feedback_file, "a", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)

            if not file_exists:
                writer.writerow([
                    "timestamp",
                    "destination_reached",
                    "helpfulness_rating",
                    "comment"
                ])

            writer.writerow([
                datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                reached_location,
                rating,
                comment
            ])

