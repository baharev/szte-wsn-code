
SELECT id, name, birthday, GROUP_CONCAT(type || ';' || date || ';' || angles, ';')

FROM

(SELECT person.id as id, person.name as name, person.birthday as birthday, 

        motion.type as type, record.date_added as date, record.angles as angles 

FROM record JOIN person ON (record.person=person.id) JOIN motion ON (record.type=motion.id)

WHERE motion.type='LEFT_ELBOW_FLEX'

ORDER BY person.name, person.birthday, motion.type, record.date_added

) inline_view

GROUP BY id

ORDER BY name, birthday;
